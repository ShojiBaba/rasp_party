// --- ライブラリのインポート ---
// ファイル操作(fs)とJSONのシリアライズ/デシリアライズ関連の機能を追加します。
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// actix_webからWebサーバー関連の機能をインポートします。
use actix_web::{web, App, HttpResponse, HttpServer,  ResponseError};
use anyhow::Result;
// serdeからシリアライズ(Serialize)とデシリアライズ(Deserialize)の機能を取り込みます。
use serde::{Deserialize, Serialize};
// tokioから非同期処理のためのメッセージチャネル(mpsc)やタスク管理機能をインポートします。
use tokio::sync::mpsc;
use tokio::task::{self, JoinHandle};

// rppalからGPIOとイベントの機能を取り込みます。
use rppal::gpio::{Event, Gpio, Level};
// bmi160クレートからセンサー制御に必要な構造体やenumをインポートします。
use bmi160::{Bmi160, SlaveAddr, SensorSelector, AccelerometerPowerMode, GyroscopePowerMode};
// linux_embedded_halからI2C通信のための機能を取り込みます。
use linux_embedded_hal::I2cdev;

// 自作ライブラリ(クレート) `rasp_party` からモジュールとトレイトをインポートします。
use rasp_party::rasp_gpio::{RaspGpio, GPIO};
use rasp_party::GpioPin;

// --- グローバルな設定とデータ構造の定義 ---

/// どのセンサーを使用するかを制御するための設定フラグ。
/// JSONファイルとの間で読み書き両方できるように`Serialize`と`Deserialize`を付与します。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct SensorConfig {
    use_ultrasonic: bool,
    use_imu: bool,
}

/// 複数スレッドで共有するための、アプリケーション全体の状態を保持するデータ構造。
/// センサーの生データと、フィルター計算によって更新され続ける融合データを両方保持します。
#[derive(Debug, Clone, Serialize)]
struct AppState {
    // --- Raw Sensor Data ---
    distance_cm: f64,
    raw_accel_x: f64,
    raw_accel_y: f64,
    raw_accel_z: f64,
    raw_gyro_x: f64,
    raw_gyro_y: f64,
    raw_gyro_z: f64,

    // --- Fused Data (Complementary Filter) ---
    fused_angle_x: f64,
    fused_angle_y: f64,
    
    // --- Internal State (JSONには含めない) ---
    // `#[serde(skip)]`を付けることで、このフィールドはJSONに変換されません。
    #[serde(skip)]
    last_update: Instant,
}

/// AppState構造体の初期値を定義します。
impl Default for AppState {
    fn default() -> Self {
        Self {
            distance_cm: -1.0,
            raw_accel_x: 0.0,
            raw_accel_y: 0.0,
            raw_accel_z: 0.0,
            raw_gyro_x: 0.0,
            raw_gyro_y: 0.0,
            raw_gyro_z: 0.0,
            fused_angle_x: 0.0,
            fused_angle_y: 0.0,
            last_update: Instant::now(),
        }
    }
}

// --- 設定ファイル読み込み・作成機能 ---

/// `gpio_sensors.json`を読み込み、`SensorConfig`構造体を返します。
/// ファイルが存在しない場合は、デフォルト設定で新規作成します。
fn load_config() -> SensorConfig {
    const CONFIG_FILE_PATH: &str = "gpio_sensors.json";
    
    // デフォルトの設定値。ファイル読み込み失敗時や新規作成時の基準となります。
    let default_config = SensorConfig {
        use_ultrasonic: true,
        use_imu: true,
    };

    match File::open(CONFIG_FILE_PATH) {
        Ok(file) => {
            // ファイルが存在する場合、内容を読み込んでパースします。
            let reader = BufReader::new(file);
            match serde_json::from_reader(reader) {
                Ok(config) => {
                    println!("[Config] Loaded configuration from {}", CONFIG_FILE_PATH);
                    config
                }
                Err(e) => {
                    // JSONの形式が不正な場合はエラーを出し、デフォルト値を使用します。
                    eprintln!("[Config] Error parsing {}: {}. Using default.", CONFIG_FILE_PATH, e);
                    default_config
                }
            }
        }
        Err(_) => {
            // ファイルが存在しない場合、デフォルト設定で新規作成します。
            println!("[Config] {} not found. Creating with default settings.", CONFIG_FILE_PATH);
            
            // `SensorConfig`を人間が読みやすい形式(インデント付き)のJSON文字列に変換します。
            let json_string = serde_json::to_string_pretty(&default_config)
                .expect("Failed to serialize default config");

            // ファイルに書き出します。
            if let Err(e) = fs::write(CONFIG_FILE_PATH, json_string) {
                eprintln!("[Config] Failed to write default config file: {}", e);
            }
            
            // 新規作成した場合でも、デフォルト設定を返してプログラムの実行を継続します。
            default_config
        }
    }
}

// --- センサー制御ロジック ---

/// 超音波センサーの距離を測定するロジック。
fn measure_distance(gpio: &Gpio, trig_pin: u8, echo_pin: u8) -> Result<f64> {
    let mut trigger = gpio.get(trig_pin)?.into_output();
    let echo = gpio.get(echo_pin)?.into_input();
    trigger.set_low();
    thread::sleep(Duration::from_micros(5));
    trigger.set_high();
    thread::sleep(Duration::from_micros(10));
    trigger.set_low();

    let wait_start = Instant::now();
    while echo.read() == Level::Low {
        if wait_start.elapsed() > Duration::from_millis(500) {
            return Err(anyhow::anyhow!("HY-SRF05: Echo timed out waiting for start"));
        }
    }
    let pulse_start = Instant::now();
    while echo.read() == Level::High {
        if pulse_start.elapsed() > Duration::from_millis(500) {
            return Err(anyhow::anyhow!("HY-SRF05: Echo timed out waiting for end"));
        }
    }
    let pulse_duration = pulse_start.elapsed();
    let distance = (pulse_duration.as_secs_f64() * 34300.0) / 2.0;
    Ok(distance)
}

/// センサー群からデータを読み取り続け、共有状態(`AppState`)を更新するループ。
/// このループは生データを取得し、共有状態に書き込むことに専念します。
fn run_sensor_loop(
    app_state_arc: Arc<Mutex<AppState>>,
    running_arc: Arc<AtomicBool>,
    config: SensorConfig,
) {
    let gpio = if config.use_ultrasonic { Some(Gpio::new().expect("...")) } else { None };
    let mut bmi160 = if config.use_imu {
        let i2c_dev = I2cdev::new("/dev/i2c-1").expect("...");
        let mut sensor = Bmi160::new_with_i2c(i2c_dev, SlaveAddr::default());
        sensor.set_accel_power_mode(AccelerometerPowerMode::Normal).expect("...");
        sensor.set_gyro_power_mode(GyroscopePowerMode::Normal).expect("...");
        thread::sleep(Duration::from_millis(100));
        Some(sensor)
    } else { None };

    println!("[Sensor Thread] Started with config: {:?}", config);
    while running_arc.load(Ordering::Relaxed) {
        // `Mutex`で保護された共有状態へのアクセス権を取得します。
        let mut state = app_state_arc.lock().unwrap();
        
        // --- 生データ取得と共有状態への書き込み ---
        if config.use_ultrasonic {
            if let Some(ref gpio_instance) = gpio {
                if let Ok(dist) = measure_distance(gpio_instance, GPIO::P7_4.as_u8(), GPIO::P11_17.as_u8()) {
                    state.distance_cm = dist;
                }
            }
        }
        
        if config.use_imu {
            if let Some(ref mut imu_instance) = bmi160 {
                let selector = SensorSelector::new().accel().gyro();
                if let Ok(data) = imu_instance.data(selector) {
                    if let Some(accel) = data.accel {
                        state.raw_accel_x = accel.x as f64;
                        state.raw_accel_y = accel.y as f64;
                        state.raw_accel_z = accel.z as f64;
                    }
                    if let Some(gyro) = data.gyro {
                        state.raw_gyro_x = gyro.x as f64;
                        state.raw_gyro_y = gyro.y as f64;
                        state.raw_gyro_z = gyro.z as f64;
                    }
                }
            }
        }
        
        // 最終更新時刻を記録します。これは相補フィルターの計算で使われます。
        state.last_update = Instant::now();
        
        // Mutexのロックをこのスコープの終わりで自動的に解放します。
        // `drop(state)`を明示的に呼び出すことで、ロックを早めに解放することも可能です。
        
        // フィルター計算の精度を上げるため、ループ周期を短く設定します。
        thread::sleep(Duration::from_millis(50));
    }
    println!("[Sensor Thread] Finished.");
}


// --- Webサーバー関連 ---

/// GETパラメータ `/sensors?alpha=0.9` を受け取るための構造体。
#[derive(Deserialize)]
struct FilterParams {
    alpha: Option<f64>,
}

/// パラメータ検証エラーのためのカスタムエラー型。
/// `ResponseError`を実装することで、actix-webが自動的にエラーレスポンスを生成してくれます。
#[derive(Debug)]
struct AppError(String);
impl std::fmt::Display for AppError { fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { write!(f, "{}", self.0) } }
impl ResponseError for AppError { fn error_response(&self) -> HttpResponse { HttpResponse::BadRequest().body(self.0.clone()) } }

/// Webからのリクエストに応じて、フィルター計算を行い、最終的なJSONを返すハンドラ。
async fn get_sensors(
    state_arc: web::Data<Arc<Mutex<AppState>>>,
    params: web::Query<FilterParams>,
) -> Result<HttpResponse, AppError> {
    // ジャイロセンサーを重視する一般的な値をデフォルトとします。
    const DEFAULT_ALPHA: f64 = 0.98;

    // 1. alphaパラメータを検証します。
    let alpha = match params.alpha {
        Some(a) if (0.0..=1.0).contains(&a) => a, // 0.0～1.0の範囲内ならOK
        Some(_) => return Err(AppError("Parameter 'alpha' must be between 0.0 and 1.0.".to_string())),
        None => DEFAULT_ALPHA, // パラメータがなければデフォルト値を使用
    };

    // 2. 共有状態をロックし、必要なデータをローカル変数にコピーします。
    let mut state = state_arc.lock().unwrap();

    // 3. 時間差(dt)を計算します。
    let now = Instant::now();
    let dt = now.duration_since(state.last_update).as_secs_f64();

    // 4. 相補フィルターの計算を実行します。
    // 加速度センサーの値から角度を計算 (ラジアン)
    let accel_angle_x = (state.raw_accel_y).atan2(state.raw_accel_z);
    let accel_angle_y = (state.raw_accel_x * -1.0).atan2((state.raw_accel_y.powi(2) + state.raw_accel_z.powi(2)).sqrt());
    
    // 前回の角度と今回のジャイロの値から新しい角度を計算します。
    // 新角度 = alpha * (前回の角度 + ジャイロ角速度 * dt) + (1 - alpha) * (加速度から求めた角度)
    let new_fused_angle_x = alpha * (state.fused_angle_x + state.raw_gyro_x * dt) + (1.0 - alpha) * accel_angle_x;
    let new_fused_angle_y = alpha * (state.fused_angle_y + state.raw_gyro_y * dt) + (1.0 - alpha) * accel_angle_y;

    // 5. 計算結果を共有状態に書き戻します (次の計算で「前回の角度」として使われます)。
    state.fused_angle_x = new_fused_angle_x;
    state.fused_angle_y = new_fused_angle_y;
    state.last_update = now; // 最終更新時刻を更新

    // 6. レスポンス用のJSONを生成します。
    #[derive(Serialize)] struct ApiResponse { distance_cm: f64, raw_data: RawData, fused_data: FusedData }
    #[derive(Serialize)] struct RawData { accel_x: f64, accel_y: f64, accel_z: f64, gyro_x: f64, gyro_y: f64, gyro_z: f64 }
    #[derive(Serialize)] struct FusedData { angle_x_deg: f64, angle_y_deg: f64 }
    
    let response_body = ApiResponse {
        distance_cm: state.distance_cm,
        raw_data: RawData { accel_x: state.raw_accel_x, accel_y: state.raw_accel_y, accel_z: state.raw_accel_z, gyro_x: state.raw_gyro_x, gyro_y: state.raw_gyro_y, gyro_z: state.raw_gyro_z },
        fused_data: FusedData { angle_x_deg: new_fused_angle_x * 180.0 / std::f64::consts::PI, angle_y_deg: new_fused_angle_y * 180.0 / std::f64::consts::PI },
    };
    
    Ok(HttpResponse::Ok().json(response_body))
}

// --- アプリケーションのメインロジック ---

/// センサー管理タスクに送るコマンドです。
#[derive(Debug)]
enum SensorCommand { Start, Stop }

/// 非同期ランタイムのエントリーポイントです。
#[actix_web::main]
async fn main() -> std::io::Result<()> {
    // 1. 設定の初期化
    let sensor_config = load_config();
    println!("Starting Sensor Demon with config: {:?}", sensor_config);
    println!("Please press the tact switch to start/stop sensor reading.");

    // 2. 共有リソースの準備
    let app_state = Arc::new(Mutex::new(AppState::default()));
    let (tx, mut rx) = mpsc::channel::<SensorCommand>(32);

    // 3. スレッド管理タスクの起動
    let app_state_for_manager = app_state.clone();
    let manager_task_handle = tokio::spawn(async move {
        let mut sensor_thread_handle: Option<JoinHandle<()>> = None;
        let running_flag = Arc::new(AtomicBool::new(false));
        println!("[Manager Task] Waiting for commands...");
        while let Some(command) = rx.recv().await {
            match command {
                SensorCommand::Start => {
                    if sensor_thread_handle.is_none() {
                        println!("[Manager Task] Received Start command.");
                        running_flag.store(true, Ordering::SeqCst);
                        let state_clone = app_state_for_manager.clone();
                        let running_clone = running_flag.clone();
                        let handle = task::spawn_blocking(move || {
                            run_sensor_loop(state_clone, running_clone, sensor_config);
                        });
                        sensor_thread_handle = Some(handle);
                    }
                }
                SensorCommand::Stop => {
                    if let Some(handle) = sensor_thread_handle.take() {
                        println!("[Manager Task] Received Stop command.");
                        running_flag.store(false, Ordering::SeqCst);
                        if let Err(e) = handle.await { eprintln!("[Manager Task] Error: {:?}", e); }
                        else { println!("[Manager Task] Thread joined successfully."); }
                    }
                }
            }
        }
    });

    // 4. タクトスイッチ監視のセットアップと開始
    let gpio = Gpio::new().map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    let handle = tokio::runtime::Handle::current();
    let mut switch_gpio = RaspGpio::create(GPIO::P38_20, gpio, {
        let tx_clone = tx.clone();
        move |switch_state: bool, _level: Event| {
            let tx = tx_clone.clone();
            async move {
                let command = if switch_state { SensorCommand::Start } else { SensorCommand::Stop };
                println!("[Callback] Sending command: {:?}", command);
                if let Err(e) = tx.send(command).await { eprintln!("[Callback] Failed: {}", e); }
            }
        }
    }).await.unwrap();
    switch_gpio.watch(handle).await.unwrap();
    
    // 5. HTTPサーバーの起動
    println!("Starting HTTP server at http://0.0.0.0:9090");
    let server = HttpServer::new(move || {
        App::new()
            .app_data(web::Data::new(app_state.clone()))
            .route("/sensors", web::get().to(get_sensors))
    })
    .bind(("0.0.0.0", 9090))?
    .run();

    // 6. 優雅なシャットダウン (Graceful Shutdown)
    server.await?;
    println!("[main] Server stopped. Cleaning up resources...");
    switch_gpio.unwatch().await.unwrap();
    let _ = tx.send(SensorCommand::Stop).await;
    manager_task_handle.await.expect("Manager task failed to shutdown");
    
    println!("[main] Application finished gracefully.");
    Ok(())
}