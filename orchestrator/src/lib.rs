use std::time::Duration;

use hardware::{
    robot_arm::{Motion, Point, RobotArm},
    wind_shape::WindShape,
};

mod hardware;

#[allow(dead_code)]
pub async fn test_robot_arm() {
    tracing::info!("Connecting to robot arm");
    let mut robot_arm = RobotArm::connect("192.168.100.254")
        .await
        .expect("Failed to connect to RobotArm");

    let mut position = Point {
        x: 500.,
        y: 0.,
        z: 400.,
        rx: 0.,
        ry: 0.,
        rz: 0.,
    };

    tracing::info!("Issuing move command");
    robot_arm.move_to(&Motion::Linear(position)).await;

    tracing::info!("Waiting for motion to complete");
    robot_arm.wait_motion().await;

    position.y = 200.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.x = 200.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    tracing::info!("Returning and stopping robot arm");
    robot_arm.return_and_stop().await;
}

#[allow(dead_code)]
pub async fn test_wind_shape() {
    let mut wind_shape = WindShape::connect("192.168.88.40")
        .await
        .expect("Failed to connect to WindShape");

    wind_shape.request_control().await;

    wind_shape.enable_power().await;
    sleep_s(1).await;

    wind_shape.set_fan_speed(10).await;
    sleep_s(3).await;

    wind_shape.set_fan_speed(15).await;
    sleep_s(1).await;

    wind_shape.set_fan_speed(20).await;
    sleep_s(1).await;

    wind_shape.set_fan_speed(5).await;
    sleep_s(1).await;

    wind_shape.disable_power().await;
    wind_shape.release_control().await;
}

async fn sleep_s(seconds: u64) {
    tokio::time::sleep(Duration::from_secs(seconds)).await;
}
