use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use actix_web::{web, App, HttpResponse, HttpServer, Responder};
use anyhow::Result;
use rasp_party::GpioPin;
use serde::Serialize;
use tokio::sync::mpsc;
use tokio::task::{self, JoinHandle};

use rppal::gpio::{Event, Gpio, Level};
use bmi160::{Bmi160, SlaveAddr, SensorSelector, AccelerometerPowerMode, GyroscopePowerMode};
use linux_embedded_hal::I2cdev;

// rasp_gpioモジュールをインポート
use rasp_party::rasp_gpio::{RaspGpio, GPIO};

// --- ADDED: センサーの使用を制御する設定フラグ ---
#[derive(Debug, Clone, Copy)]
struct SensorConfig {
    use_ultrasonic: bool,
    use_imu: bool,
}

// --- データ構造の定義 (AppSensorData に名前を統一) ---
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

impl Default for AppSensorData {
    fn default() -> Self {
        Self {
            distance_cm: -1.0, // 未使用時は-1.0
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

// --- センサー制御ロジック (変更なし) ---
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

/// センサー読み取りループを実行する関数
// CHANGED: SensorConfigを引数に追加
fn run_sensor_loop(
    sensor_data_arc: Arc<Mutex<AppSensorData>>,
    running_arc: Arc<AtomicBool>,
    config: SensorConfig,
) {
    // --- センサーの遅延初期化 ---
    // 各センサーは使用する場合のみ初期化される
    let gpio = if config.use_ultrasonic {
        Some(Gpio::new().expect("Failed to initialize GPIO"))
    } else {
        None
    };

    let mut bmi160 = if config.use_imu {
        let i2c_dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C");
        let mut sensor = Bmi160::new_with_i2c(i2c_dev, SlaveAddr::default());
        sensor.set_accel_power_mode(AccelerometerPowerMode::Normal)
              .expect("Failed to set accelerometer power mode");
        sensor.set_gyro_power_mode(GyroscopePowerMode::Normal)
              .expect("Failed to set gyroscope power mode");
        thread::sleep(Duration::from_millis(100));
        Some(sensor)
    } else {
        None
    };

    println!("[Sensor Thread] Thread started. Reading sensors with config: {:?}", config);
    while running_arc.load(Ordering::Relaxed) {
        let mut current_data = AppSensorData::default();

        // --- CHANGED: フラグに基づいて条件付きでセンサーを読み取る ---
        if config.use_ultrasonic {
            if let Some(ref gpio_instance) = gpio {
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
        *sensor_data_arc.lock().unwrap() = current_data;
        thread::sleep(Duration::from_millis(1000));
    }
    println!("[Sensor Thread] Thread finished.");
}


// --- Webサーバー ---
async fn get_sensors(data: web::Data<Arc<Mutex<AppSensorData>>>) -> impl Responder {
    let data_lock = data.lock().unwrap();
    HttpResponse::Ok().json(&*data_lock)
}

// --- スレッド管理タスクへのコマンド ---
#[derive(Debug)]
enum SensorCommand {
    Start,
    Stop,
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    // --- ADDED: ここでどのセンサーを使うか設定します ---
    let sensor_config = SensorConfig {
        use_ultrasonic: true, // 超音波センサーを使うか
        use_imu: false,        // 6軸IMUを使うか
    };
    
    println!("Starting Sensor Demon with config: {:?}", sensor_config);
    println!("Please press the tact switch to start/stop sensor reading.");

    // --- 共有データの準備 ---
    let sensor_data = Arc::new(Mutex::new(AppSensorData::default()));

    // --- スレッド管理のためのメッセージチャネルを作成 ---
    let (tx, mut rx) = mpsc::channel::<SensorCommand>(32);

    let sensor_data_for_manager = sensor_data.clone();
    
    // --- スレッド管理を専門に行うタスクを起動 ---
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
                        
                        // CHANGED: センサー設定をスレッドに渡す
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

    // --- タクトスイッチのセットアップ ---
    let gpio = Gpio::new().map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    let handle = tokio::runtime::Handle::current();
    
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
    
    // --- HTTPサーバーの起動 ---
    println!("Starting HTTP server at http://0.0.0.0:9090");
    let server = HttpServer::new(move || {
        App::new()
            .app_data(web::Data::new(sensor_data.clone()))
            .route("/sensors", web::get().to(get_sensors))
    })
    .bind(("0.0.0.0", 9090))?
    .run();

    server.await?;
    
    // --- クリーンアップ処理 ---
    println!("Server stopped. Cleaning up resources...");
    switch_gpio.unwatch().await.unwrap();
    let _ = tx.send(SensorCommand::Stop).await;
    manager_task_handle.await.expect("Manager task failed to shutdown");
    
    println!("Application finished gracefully.");
    Ok(())
}