use std::{io, net::IpAddr, time::Duration};

use crate::{
    defs::Point,
    hardware::{
        robot_arm::{Motion, MotionConfig, RobotArm},
        wind_shape::WindShape,
    },
};

pub async fn run(robot_ip: IpAddr, robot_port: u16, windshape_ip: IpAddr) -> io::Result<()> {
    test_robot_arm(robot_ip, robot_port).await;
    test_wind_shape(windshape_ip).await;
    Ok(())
}

async fn test_robot_arm(ip: IpAddr, port: u16) {
    tracing::info!("Connecting to robot arm");

    let mut robot_arm = RobotArm::connect(ip, port, None)
        .await
        .expect("Failed to connect to RobotArm");

    let mut position = Point {
        x: 500.,
        y: 100.,
        z: 400.,
        rx: 0.,
        ry: 0.,
        rz: 0.,
    };

    let mut offset = Point {
        x: 660.,
        z: 20.,
        ..Default::default()
    };

    let mut config = MotionConfig {
        rotation: 10000.,
        translation: 10000.,
        acceleration_scale: 20,
        deceleration_scale: 20,
        speed_scale: 100,
    };

    robot_arm.set_offset(&offset).await;
    robot_arm.set_config(&config).await;

    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.x = 1200.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    offset.z = 100.;
    config.acceleration_scale = 100;
    config.deceleration_scale = 100;

    robot_arm.set_offset(&offset).await;
    robot_arm.set_config(&config).await;

    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rx = 90.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rx = -90.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rx = 0.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    offset.z = 0.;
    robot_arm.set_offset(&offset).await;

    position.x = 500.;
    position.z = 450.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    config.acceleration_scale = 15;
    config.deceleration_scale = 15;
    config.rotation = 30.;
    robot_arm.set_config(&config).await;

    position.rz = -45.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rz = 20.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    config.rotation = 10000.;
    config.translation = 10000.;

    robot_arm.return_home().await;
    robot_arm.wait_motion().await;
}

async fn test_wind_shape(ip: IpAddr) {
    let mut wind_shape = WindShape::connect(ip)
        .await
        .expect("Failed to connect to WindShape");

    wind_shape.request_control().await;

    wind_shape.enable_power().await;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(10).await;
    sleep_s(3.).await;

    wind_shape.set_fan_speed(15).await;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(20).await;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(5).await;
    sleep_s(1.).await;

    wind_shape.disable_power().await;
    wind_shape.release_control().await;
}

async fn sleep_s(seconds: f32) {
    tokio::time::sleep(Duration::from_secs_f32(seconds)).await;
}
