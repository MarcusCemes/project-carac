use std::{io, time::Duration};

use tokio::{fs, time::sleep};

use crate::{
    config::Config,
    hardware::{
        load_cell::LoadCell, motion_capture::MotionCapture, robot_arm::RobotArm,
        wind_shape::WindShape,
    },
    recording::Recorder,
};

pub async fn launch(config_path: &str) -> io::Result<()> {
    let config: Config = {
        let data = fs::read(config_path).await?;

        serde_yaml::from_slice(&data).map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Failed to parse config: {e}"),
            )
        })?
    };

    let ctx = Context::create(&config).await;
    let recorder = Recorder::new();

    if let Some((mc, cfg)) = &ctx.motion_capture.zip(config.hardware.motion_capture) {
        for rb in &cfg.rigid_bodies {
            mc.subscribe(rb, &recorder).await;
        }
    }

    if let Some(lc) = &ctx.load_cell {
        lc.subscribe(&recorder).await;
        lc.set_bias().await?;
        lc.start_streaming().await?;
    }

    recorder.clear_buffer().await;
    recorder.reset_reference_time().await;
    recorder.start_recording();

    sleep(Duration::from_secs_f32(2.)).await;

    recorder.stop_recording();

    if let Some(lc) = &ctx.load_cell {
        lc.stop_streaming().await?;
    }

    recorder.complete().await.encode(&mut io::stdout())?;

    Ok(())
}

struct Context {
    motion_capture: Option<MotionCapture>,
    load_cell: Option<LoadCell>,
    wind_shape: Option<WindShape>,
    robot_arm: Option<RobotArm>,
}

impl Context {
    async fn create(config: &Config) -> Self {
        Context {
            motion_capture: match &config.hardware.motion_capture {
                Some(cfg) => Some(
                    MotionCapture::connect(cfg.ip, cfg.multicast_ip)
                        .await
                        .expect("Failed to connect to motion capture"),
                ),
                None => None,
            },

            load_cell: match &config.hardware.load_cell {
                Some(cfg) => Some(
                    LoadCell::connect(cfg.ip)
                        .await
                        .expect("Failed to connect to load cell"),
                ),
                None => None,
            },

            robot_arm: match &config.hardware.robot_arm {
                Some(cfg) => Some(
                    RobotArm::connect(cfg.ip, cfg.port, None)
                        .await
                        .expect("Failed to connect to robot arm"),
                ),
                None => None,
            },

            wind_shape: match &config.hardware.wind_shape {
                Some(wind_shape) => Some(
                    WindShape::connect(wind_shape.ip)
                        .await
                        .expect("Failed to connect to wind shape"),
                ),
                None => None,
            },
        }
    }
}
