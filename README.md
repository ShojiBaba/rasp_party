# Rasp Party (`rasp_party`)

## 概要

Raspberry PiのGPIO操作を補助するためのRust製ライブラリ（クレート）です。
このライブラリは、`gpio-sensors`のようなアプリケーションを開発する際に、GPIOの管理をより簡単かつ直感的に行うことを目的としています。

このクレートは単体で実行するアプリケーションではなく、他のRustプロジェクトから依存関係として利用します。

## 主な機能

- **ピン定義の抽象化**
  - GPIOピン番号を、物理的なピン配置や機能に基づいた分かりやすい名前（Enum型）で定義しているため、コードの可読性が向上します。
    - `GPIO::P7_4`, `I2C::P3_2SDA1` のように指定できます。

- **スイッチ入力のイベント管理**
  - タクトスイッチなどのGPIO入力に対して、以下の機能を提供します。
    - **チャタリング防止:** 短時間に発生する微細なON/OFFを無視します。
    - **非同期コールバック:** スイッチの状態が変化した際に、指定した非同期処理（`async`ブロック）を実行します。

## 使用方法

`Cargo.toml`の`[dependencies]`セクションに、このクレートへのパスを追加します。

```toml
[dependencies]
rasp_party = { path = "../rasp_party" }
```

## 使用例

`GPIO 20` に接続されたタクトスイッチの状態を監視し、押されるたびにコンソールにメッセージを出力する例です。

```rust
use rasp_party::{rasp_gpio::{RaspGpio, GPIO}, GpioPin};
use rppal::gpio::{Gpio, Event};
use tokio::runtime::Handle;
use std::error::Error;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // rppalクレートからGpioインスタンスを初期化
    let gpio = Gpio::new()?;
    
    // 現在のTokioランタイムへのハンドルを取得
    let handle = Handle::current();

    // `RaspGpio`インスタンスを作成
    // - GPIO 20 をスイッチとして使用
    // - スイッチの状態が変わったときに実行する非同期コールバックを登録
    let mut switch_gpio = RaspGpio::create(GPIO::P38_20, gpio, move |switch_state: bool, _level: Event| {
        async move {
            if switch_state {
                println!("Switch is now ON");
            } else {
                println!("Switch is now OFF");
            }
        }
    }).await.unwrap();

    // スイッチの監視を開始
    switch_gpio.watch(handle).await?;
    println!("Watching for switch events. Press Ctrl+C to exit.");

    // アプリケーションのメインループ (Ctrl+Cが押されるまで待機)
    tokio::signal::ctrl_c().await?;

    // アプリケーション終了前に、監視をクリーンアップ
    switch_gpio.unwatch().await?;
    println!("Stopped watching.");

    Ok(())
}
```
