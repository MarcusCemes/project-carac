use std::time::Duration;

use clap::Parser;

use hardware::{
    robot_arm::{MoveType, Position, RobotArm},
    wind_shape::WindShape,
};

mod hardware;

#[derive(Parser)]
struct Cli {}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    tracing_subscriber::fmt::init();

    test_robot_arm().await;
    test_windshape().await;
}

#[allow(dead_code)]
async fn test_robot_arm() {
    tracing::info!("Connecting to robot arm");
    let mut robot_arm = RobotArm::connect("192.168.100.254")
        .await
        .expect("Failed to connect to RobotArm");

    let mut position = Position {
        x: 500.,
        y: 0.,
        z: 400.,
        rx: 0.,
        ry: 0.,
        rz: 0.,
    };

    tracing::info!("Issuing move command");
    robot_arm.move_position(position, MoveType::Linear).await;

    tracing::info!("Waiting for motion to complete");
    robot_arm.wait_motion().await;

    position.y = 200.;
    robot_arm.move_position(position, MoveType::Linear).await;
    robot_arm.wait_motion().await;

    position.x = 200.;
    robot_arm.move_position(position, MoveType::Linear).await;
    robot_arm.wait_motion().await;

    tracing::info!("Returning and stopping robot arm");
    robot_arm.return_and_stop().await;
}

#[allow(dead_code)]
async fn test_windshape() {
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
