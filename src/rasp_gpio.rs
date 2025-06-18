
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::{error::Error, sync::Arc};
// use std::thread;
use std::time::{Duration, Instant};
use rppal::gpio::{Event, Gpio, InputPin, Trigger};
use futures::future::BoxFuture;
use tokio::sync::Mutex;
use tokio::runtime::Handle;
use tokio::task::{self, JoinHandle};
// use tokio::time::{sleep};

/// u8として値を取得できることを示すトレイト
pub trait GpioPin {
    fn as_u8(&self) -> u8;
}
// エラーを表すための空の構造体
#[derive(Debug, PartialEq)]
pub struct InvalidStatus(u8);

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum I2C {
    P3_2SDA1 = 2,
    P5_3SCL1 = 3,
}

// I2C enumにGpioPinトレイトを実装
impl GpioPin for I2C {
    
    fn as_u8(&self) -> u8 {
        // *selfで値を取得し、u8にキャストする
        *self as u8
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum GPIO {
    P7_4 = 4,
    P11_17 = 17,
    P13_27 = 27,
    P15_22 = 22,
    P16_23 = 23,
    P18_24 = 24,
    P22_25 = 25,
    P29_5 = 5,
    P31_5 = 6,
    P36_16 = 16,
    P37_26 = 26,
    P38_20 = 20,
    P40_21 = 21,
}

// GPIO enumにGpioPinトレイトを実装
impl GpioPin for GPIO {
    fn as_u8(&self) -> u8 {
        // *selfで値を取得し、u8にキャストする
        *self as u8
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum UART {
    P8_14TXD = 4,
    P10_15RXD = 15,
}

// UART enumにGpioPinトレイトを実装
impl GpioPin for UART {
    fn as_u8(&self) -> u8 {
        // *selfで値を取得し、u8にキャストする
        *self as u8
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum SPI {
    P19_10MOSI = 10,
    P21_9MISO = 9,
    P23_11SCLK = 11,
    P24_8CE0 = 8,
    P26_7CE1 = 7,
}

// SPI enumにGpioPinトレイトを実装
impl GpioPin for SPI {
    fn as_u8(&self) -> u8 {
        // *selfで値を取得し、u8にキャストする
        *self as u8
    }
}

// FnMut: 環境の変数を変更する可能性があるクロージャ
// () -> BoxFuture<'static, ()>: 引数を取らず、Box化されたFutureを返す
// Send + Sync: スレッド間で安全にやり取りできる
// Arc: 複数の所有者を持つことができるスマートポインタ
pub type SwitchCallback = Arc<Mutex<dyn FnMut(bool,Event) -> BoxFuture<'static, ()> + Send + Sync>>;

// チャタリング防止のための待機時間 (ミリ秒)
const DEBOUNCE_DURATION_MS: u64 = 200;

/// Provides access to the Raspberry Pi's GPIO peripheral.
pub struct RaspGpio {
    pub switch:u8,
    // Gpio構造体を初期化
    pub gpio:Gpio,
    // 登録されたコールバックを保持する（Mutexで非同期アクセスに対応）
    callback: Option<SwitchCallback>,
    // 監視スレッドの実行状態を管理するフラグ
    running: Option<Arc<AtomicBool>>,
    // 起動したバックグラウンドタスクのハンドル
    task_handle: Option<JoinHandle<()>>,
}

impl RaspGpio {

    /// 非同期コールバックを登録するメソッド
    pub async fn create<T: GpioPin, C, F>(switch:T, gpio:Gpio, mut closure: C) -> Result<RaspGpio, ()>
    where
        C: FnMut(bool, Event) -> F + Send + Sync + 'static,
        F: std::future::Future<Output = ()> + Send + 'static,
    {

        // let callback = Arc::new(Mutex::new(move |switch: bool| {
        //     Box::pin(closure(switch))
        // }));
        let callback = Arc::new(Mutex::new(
            move |switch: bool, level: Event| -> BoxFuture<'static, ()> { // <- 型アノテーションを追加！
                Box::pin(closure(switch, level))
            },
        ));

        let rg = RaspGpio {
            switch:switch.as_u8(),
            gpio,
            callback: Some(callback),
            running: None,      // 初期状態はNone
            task_handle: None,  // 初期状態はNone
        };

        Ok(rg)
    }

    pub async fn watch(&mut self, handle: Handle) -> Result<(), Box<dyn Error>> {
        // すでに監視中の場合は何もしない
        if self.task_handle.is_some() {
            println!("[watch] 既に監視中です。");
            return Ok(());
        }

        let mut switch_pin = self.gpio.get(self.switch)?.into_input_pullup();

        // ロックしてコールバックのクローンを取得（Arcなのでクローンは軽量）
        // let callback_clone = self.callback.clone();
        let cb = self.callback.clone().ok_or("Callback is not set")?;

        // 実行中フラグを作成し、自身のフィールドにも保存する
        let running = Arc::new(AtomicBool::new(true));
        self.running = Some(running.clone());

        // --- 状態管理変数の設定 ---
        // Arc: 複数スレッド間で安全に所有権を共有するためのスマートポインタ
        // AtomicBool: 複数thredからロックなしで安全に読み書きできるブール値
        let is_watching = Arc::new(AtomicBool::new(false));

        // チャタリング対策用の、最後にスイッチが押された時刻を記録する変数
        // Mutex: 複数スレッドから安全に値を書き換えるための排他ロック
        let last_press_time = Arc::new(Mutex::new(Instant::now()));

        // // コールバックが存在する場合のみタスクを起動
        // if let Some( cb) = callback_clone {

        //     switch_pin.set_async_interrupt(Trigger::FallingEdge, Some(Duration::from_millis(100)), move |level| {

                
                // // fetch_xorは変更前の値を返す

                // // println!("スイッチが押されました！ 電源: {:?} : {:?}", state, level);
                
                // // if !previous_state {
                // //     println!("スイッチON: 点滅を開始します。");
                // // } else {
                // //     println!("スイッチOFF: 点滅を停止します。");
                // // }

                
                
                // let cb_for_task = Arc::clone(&cb);
                // let last_press_time_for_task = Arc::clone(&last_press_time);
                // let interrupt_is_watching = Arc::clone(&is_watching);
                
                // // tokio::spawnで新しい非同期タスクを生成する
                // // これにより、このメソッド自体は待機せずにすぐに完了する
                // task::spawn_blocking(move || {

                //     let should_run_callback;
                //     let mut previous_state = false;
                //     // --- クリティカルセクション（ロックが必要な範囲）をスコープで囲む ---
                //     {
                //         let mut last_time = last_press_time_for_task.lock().unwrap(); // ロック取得
                //         if last_time.elapsed() > Duration::from_millis(DEBOUNCE_DURATION_MS) {
                //             // 時刻をすぐに更新
                //             *last_time = Instant::now();
                //             previous_state = interrupt_is_watching.fetch_xor(true, Ordering::SeqCst);
                //             should_run_callback = true;
                //         } else {
                //             should_run_callback = false;
                //         }
                //     } // <- このスコープを抜けた瞬間に`last_time`が破棄され、ロックが自動的に解放される

                //     // --- ロックが解放された安全な状態で非同期処理を実行 ---
                //     if should_run_callback {
                //         println!("[Task] スイッチを検知。コールバックを実行します。");
                //         let mut cb_guard = cb_for_task.lock().await;

                //         (*cb_guard)(!previous_state, level).await;
                //         println!("[Task] コールバックの実行が完了しました。");
                //     }

                // });

            // })?;

            // tokio::signal::ctrl_c().await?;

                // 同期的な処理をブロッキングタスクとして起動する
            let join_handle = task::spawn_blocking(move || {
                println!("[Blocking Task] 割り込み設定を開始します。");
    
               // ブロッキングタスク内で割り込みを設定
               if let Err(e) = switch_pin.set_async_interrupt(Trigger::FallingEdge, Some(Duration::from_millis(100)), move |_level| {
                   // handleはCopyなので、moveでキャプチャした後もコールバック内で使用できる
                   let cb_for_task = Arc::clone(&cb);
                   let last_press_time_for_task = Arc::clone(&last_press_time);
                   let interrupt_is_watching = Arc::clone(&is_watching);
    
                   // コールバックから非同期タスクを起動
                   handle.spawn(async move {
                       // ... (以前の回答のspawn内のロジックはここ) ...
                       let should_run_callback;
                       let mut previous_state = false;
                       {
                           let mut last_time = last_press_time_for_task.lock().await;
                           if last_time.elapsed() > Duration::from_millis(DEBOUNCE_DURATION_MS) {
                               *last_time = Instant::now();
                               previous_state = interrupt_is_watching.fetch_xor(true, Ordering::SeqCst);
                               should_run_callback = true;
                           } else {
                               should_run_callback = false;
                           }
                       }
    
                       if should_run_callback {
                            let mut cb_guard = cb_for_task.lock().await;
                            // 2. ガードをデリファレンス(*)して、中のFnMutなコールバックを直接呼び出す
                            //    if let Some(...) や .as_mut() は不要
                            (*cb_guard)(!previous_state, _level).await;
                       }
                   });
               }) {
                   eprintln!("[Blocking Task] 割り込み設定に失敗: {}", e);
                   return; // エラーならスレッド終了
               }
    
               println!("[Blocking Task] 割り込み設定完了。終了シグナルを待ちます。");
                while running.load(Ordering::SeqCst) {
                    thread::sleep(Duration::from_millis(100));
                }
                println!("[Blocking Task] 終了シグナルを検知。スレッドをクリーンアップします。");
            });

            // タスクハンドルを自身のフィールドに保存
            self.task_handle = Some(join_handle);

            println!("[watch] spawn_blocking呼び出し完了、watchメソッド終了");
        // } else {
        //     println!("[EventManager] 警告: コールバックが登録されていません。");
        // }
        
        Ok(())
    }


    /// 監視を終了させるメソッド
    pub async fn unwatch(&mut self) -> Result<(), Box<dyn Error>> {
        println!("[unwatch] 監視の終了処理を開始します...");

        // 1. バックグラウンドタスクに終了を通知する
        if let Some(running) = &self.running {
            running.store(false, Ordering::SeqCst);
        }

        // 2. バックグラウンドタスクが完了するのを待つ
        // .take()で所有権を奪い、元のフィールドをNoneにする
        if let Some(handle) = self.task_handle.take() {
            handle.await?;
            println!("[unwatch] バックグラウンドタスクが正常に終了しました。");
        } else {
            println!("[unwatch] 監視タスクは実行されていませんでした。");
        }
        
        // 3. 状態をリセット
        self.running = None;

        Ok(())
    }
}