use std::{net::IpAddr, time::Duration};

use eyre::{Context, Result};

use crate::{
    defs::Point,
    hardware::{
        robot_arm::{Motion, MotionType, RobotArm, SpeedProfile},
        wind_shape::WindShape,
    },
};

pub async fn run(robot_ip: IpAddr, robot_port: u16, windshape_ip: IpAddr) -> Result<()> {
    test_robot_arm(robot_ip, robot_port).await?;
    test_wind_shape(windshape_ip).await;
    Ok(())
}

async fn test_robot_arm(ip: IpAddr, port: u16) -> Result<()> {
    tracing::info!("Connecting to robot arm");

    let robot_arm = RobotArm::connect(ip, port)
        .await
        .wrap_err("Failed to connect to RobotArm")?;

    let mut r = robot_arm.controller();

    let mut p = Point {
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

    let mut profile = SpeedProfile {
        rotation_limit: 10000.,
        translation_limit: 10000.,
        acceleration_scale: 20,
        deceleration_scale: 20,
        velocity_scale: 100,
    };

    r.set_offset(&offset).await?;
    r.set_profile(&profile).await?;

    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    p.x = 1200.;
    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    offset.z = 100.;
    profile.acceleration_scale = 100;
    profile.deceleration_scale = 100;

    r.set_offset(&offset).await?;
    r.set_profile(&profile).await?;

    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    p.rx = 90.;
    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    p.rx = -90.;
    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    p.rx = 0.;
    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    offset.z = 0.;
    r.set_offset(&offset).await?;

    p.x = 500.;
    p.z = 450.;
    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    profile.acceleration_scale = 15;
    profile.deceleration_scale = 15;
    profile.rotation_limit = 30.;
    r.set_profile(&profile).await?;

    p.rz = -45.;
    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    p.rz = 20.;
    r.move_to(Motion::Linear(&p)).await?;
    r.wait_settled().await;

    profile.rotation_limit = 10000.;
    profile.translation_limit = 10000.;

    r.go_home(MotionType::Direct).await?;
    r.wait_settled().await;

    Ok(())
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
