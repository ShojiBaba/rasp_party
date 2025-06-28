// --- ライブラリのインポート ---
// 非同期処理、Webフレームワーク、スレッド間通信、シリアライズなど、必要な機能を読み込みます。
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use actix_web::{web, App, HttpResponse, HttpServer, Responder};
use anyhow::Result;
use serde::Serialize;
use tokio::sync::mpsc;
use tokio::task::{self, JoinHandle};

use rppal::gpio::{Event, Gpio, Level};
use bmi160::{Bmi160, SlaveAddr, SensorSelector, AccelerometerPowerMode, GyroscopePowerMode};
use linux_embedded_hal::I2cdev;

// 自作ライブラリ(クレート) `rasp_party` からモジュールとトレイトをインポートします。
use rasp_party::rasp_gpio::{RaspGpio, GPIO};
use rasp_party::GpioPin;


// --- グローバルな設定とデータ構造の定義 ---

/// どのセンサーを使用するかを制御するための設定フラグ。
/// main関数の開始地点でこの値を変更するだけで、動作を簡単に切り替えられます。
#[derive(Debug, Clone, Copy)]
struct SensorConfig {
    use_ultrasonic: bool,
    use_imu: bool,
}

/// 全てのセンサーデータを集約し、JSONで外部に提供するためのデータ構造。
#[derive(Debug, Clone, Serialize)]
struct AppSensorData {
    distance_cm: f64,
    acceleration_x: f64,
    acceleration_y: f64,
    acceleration_z: f64,
    gyroscope_x: f64,
    gyroscope_y: f64,
    gyroscope_z: f64,
    timestamp: u64,
}

/// センサーデータ構造体の初期値を定義します。
impl Default for AppSensorData {
    fn default() -> Self {
        Self {
            distance_cm: -1.0, // データがないことを示すために-1.0を初期値とします。
            acceleration_x: 0.0,
            acceleration_y: 0.0,
            acceleration_z: 0.0,
            gyroscope_x: 0.0,
            gyroscope_y: 0.0,
            gyroscope_z: 0.0,
            timestamp: 0,
        }
    }
}

// --- センサー制御ロジック ---

/// 超音波センサー(HY-SRF05)の距離を測定する純粋なロジック。
fn measure_distance(gpio: &Gpio, trig_pin: u8, echo_pin: u8) -> Result<f64> {
    // Trigピンから短いパルスを送信
    let mut trigger = gpio.get(trig_pin)?.into_output();
    let echo = gpio.get(echo_pin)?.into_input();
    trigger.set_low();
    thread::sleep(Duration::from_micros(5));
    trigger.set_high();
    thread::sleep(Duration::from_micros(10));
    trigger.set_low();

    // Echoピンからの応答パルスの長さを計測
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

    // パルスの長さと音速から距離を計算 (cm)
    let distance = (pulse_duration.as_secs_f64() * 34300.0) / 2.0;
    Ok(distance)
}

/// 実際にセンサー群からデータを読み取り続けるループ処理。
/// この関数は同期的(ブロッキング)な処理であるため、`tokio::task::spawn_blocking`で実行されます。
fn run_sensor_loop(
    sensor_data_arc: Arc<Mutex<AppSensorData>>,
    running_arc: Arc<AtomicBool>,
    config: SensorConfig,
) {
    // --- センサーの遅延初期化 ---
    // `SensorConfig`に基づき、使用するセンサーのみを初期化します。
    let gpio = if config.use_ultrasonic {
        Some(Gpio::new().expect("Failed to initialize GPIO"))
    } else {
        None
    };

    let mut bmi160 = if config.use_imu {
        let i2c_dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C");
        let mut sensor = Bmi160::new_with_i2c(i2c_dev, SlaveAddr::default());
        sensor.set_accel_power_mode(AccelerometerPowerMode::Normal).expect("Failed to set accel power");
        sensor.set_gyro_power_mode(GyroscopePowerMode::Normal).expect("Failed to set gyro power");
        thread::sleep(Duration::from_millis(100)); // センサーの起動を待機
        Some(sensor)
    } else {
        None
    };

    println!("[Sensor Thread] Thread started. Reading sensors with config: {:?}", config);
    // `running_arc`フラグがtrueである限り、ループを続けます。
    while running_arc.load(Ordering::Relaxed) {
        let mut current_data = AppSensorData::default();

        // --- フラグに基づいて条件付きでセンサーを読み取る ---
        if config.use_ultrasonic {
            if let Some(ref gpio_instance) = gpio {
                // `as_u8()`は自作の`GpioPin`トレイトから来ています。
                if let Ok(dist) = measure_distance(gpio_instance, GPIO::P7_4.as_u8(), GPIO::P11_17.as_u8()) {
                    current_data.distance_cm = dist;
                    println!("距離:{:?}cm", dist);
                }
            }
        }
        
        if config.use_imu {
            if let Some(ref mut imu_instance) = bmi160 {
                let selector = SensorSelector::new().accel().gyro();
                if let Ok(data) = imu_instance.data(selector) {
                    if let Some(accel) = data.accel {
                        current_data.acceleration_x = accel.x as f64;
                        current_data.acceleration_y = accel.y as f64;
                        current_data.acceleration_z = accel.z as f64;
                    }
                    if let Some(gyro) = data.gyro {
                        current_data.gyroscope_x = gyro.x as f64;
                        current_data.gyroscope_y = gyro.y as f64;
                        current_data.gyroscope_z = gyro.z as f64;
                    }
                }
            }
        }
        
        current_data.timestamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs();
        // `Mutex`で保護された共有データに最新の値を書き込みます。
        *sensor_data_arc.lock().unwrap() = current_data;
        thread::sleep(Duration::from_millis(1000)); // 1秒ごとに処理
    }
    println!("[Sensor Thread] Thread finished.");
}


// --- Webサーバー関連 ---

/// Webブラウザ等からのリクエストに応じて、最新のセンサーデータをJSONで返すハンドラ。
async fn get_sensors(data: web::Data<Arc<Mutex<AppSensorData>>>) -> impl Responder {
    let data_lock = data.lock().unwrap();
    HttpResponse::Ok().json(&*data_lock)
}

// --- アプリケーションのメインロジック ---

/// センサー管理タスクに送るコマンドを定義。
#[derive(Debug)]
enum SensorCommand {
    Start,
    Stop,
}

/// 非同期ランタイムのエントリーポイント。
#[actix_web::main]
async fn main() -> std::io::Result<()> {
    // 1. 設定の初期化
    let sensor_config = SensorConfig {
        use_ultrasonic: true, // trueにすると超音波センサーを使用
        use_imu: false,       // trueにすると6軸IMUを使用
    };
    
    println!("Starting Sensor Demon with config: {:?}", sensor_config);
    println!("Please press the tact switch to start/stop sensor reading.");

    // 2. 共有リソースの準備
    // 全タスクで共有するデータ領域。`Arc<Mutex<...>>`はスレッドセーフな共有の定石。
    let sensor_data = Arc::new(Mutex::new(AppSensorData::default()));
    // タスク間でコマンドをやり取りするための非同期チャネル。
    let (tx, mut rx) = mpsc::channel::<SensorCommand>(32);

    // 3. スレッド管理タスクの起動
    // HTTPサーバーやスイッチ監視とは独立した、バックグラウンドで動作するタスク。
    // センサー読み取りスレッドの起動・停止に全ての責任を持つ。
    let sensor_data_for_manager = sensor_data.clone();
    let manager_task_handle = tokio::spawn(async move {
        let mut sensor_thread_handle: Option<JoinHandle<()>> = None;
        let running_flag = Arc::new(AtomicBool::new(false));

        println!("[Manager Task] Waiting for commands...");
        while let Some(command) = rx.recv().await {
            match command {
                SensorCommand::Start => {
                    if sensor_thread_handle.is_none() {
                        println!("[Manager Task] Received Start command. Starting thread...");
                        running_flag.store(true, Ordering::SeqCst);
                        let data_clone = sensor_data_for_manager.clone();
                        let running_clone = running_flag.clone();
                        let handle = task::spawn_blocking(move || {
                            run_sensor_loop(data_clone, running_clone, sensor_config);
                        });
                        sensor_thread_handle = Some(handle);
                    }
                }
                SensorCommand::Stop => {
                    if let Some(handle) = sensor_thread_handle.take() {
                        println!("[Manager Task] Received Stop command. Stopping thread...");
                        running_flag.store(false, Ordering::SeqCst);
                        if let Err(e) = handle.await {
                            eprintln!("[Manager Task] Error joining thread: {:?}", e);
                        } else {
                            println!("[Manager Task] Thread joined successfully.");
                        }
                    }
                }
            }
        }
    });

    // 4. タクトスイッチ監視のセットアップと開始
    let gpio = Gpio::new().map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    let handle = tokio::runtime::Handle::current();
    
    // スイッチが押されたら、コマンドをチャネル(`tx`)経由で管理タスクに送信するだけ。
    let mut switch_gpio = RaspGpio::create(GPIO::P38_20, gpio, {
        let tx_clone = tx.clone();
        move |switch_state: bool, _level: Event| {
            let tx = tx_clone.clone();
            async move {
                let command = if switch_state { SensorCommand::Start } else { SensorCommand::Stop };
                println!("[Callback] Sending command: {:?}", command);
                if let Err(e) = tx.send(command).await {
                    eprintln!("[Callback] Failed to send command: {}", e);
                }
            }
        }
    }).await.unwrap();
    switch_gpio.watch(handle).await.unwrap();
    
    // 5. HTTPサーバーの起動
    println!("Starting HTTP server at http://0.0.0.0:9090");
    let server = HttpServer::new(move || {
        App::new()
            .app_data(web::Data::new(sensor_data.clone()))
            .route("/sensors", web::get().to(get_sensors))
    })
    .bind(("0.0.0.0", 9090))?
    .run();

    // 6. 優雅なシャットダウン (Graceful Shutdown)
    server.await?; // Ctrl+Cなどでサーバーが停止するまで待機
    
    println!("[main] Server stopped. Cleaning up resources...");
    switch_gpio.unwatch().await.unwrap(); // スイッチ監視を停止
    let _ = tx.send(SensorCommand::Stop).await; // 念のため停止コマンドを送信
    manager_task_handle.await.expect("Manager task failed to shutdown"); // 管理タスクの完全終了を待つ
    
    println!("[main] Application finished gracefully.");
    Ok(())
}