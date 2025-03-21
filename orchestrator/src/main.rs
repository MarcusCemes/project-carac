use std::time::Duration;

use clap::Parser;

use hardware::wind_shape::WindShape;

mod hardware;

#[derive(Parser)]
struct Cli {}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let mut wind_shape = WindShape::connect("192.168.88.40")
        .await
        .expect("Failed to connect to WindShape");

    wind_shape.request_control().await;

    wind_shape.enable_power().await;
    sleep(1).await;

    wind_shape.set_fan_speed(10).await;
    sleep(3).await;

    wind_shape.set_fan_speed(15).await;
    sleep(1).await;

    wind_shape.set_fan_speed(20).await;
    sleep(1).await;

    wind_shape.set_fan_speed(5).await;
    sleep(1).await;

    wind_shape.disable_power().await;
    wind_shape.release_control().await;
}

async fn sleep(seconds: u64) {
    tokio::time::sleep(Duration::from_secs(seconds)).await;
}
