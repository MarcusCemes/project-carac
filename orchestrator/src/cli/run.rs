use std::{io, time::Duration};

use eyre::{Context, Result};
use tokio::{fs, time::sleep};

use crate::{
    config::Config,
    hardware::{
        load_cell::LoadCell, motion_capture::MotionCapture, robot_arm::RobotArm,
        wind_shape::WindShape,
    },
    recording::Sink,
};

pub async fn launch(config_path: &str) -> Result<()> {
    let config = {
        let data = fs::read(config_path).await?;
        Config::load(&data)?
    };

    let ctx = HardwareContext::create(&config).await?;
    let sink = Sink::new();

    if let Some((mc, cfg)) = &ctx.motion_capture.zip(config.hardware.motion_capture) {
        for rb in &cfg.rigid_bodies {
            mc.subscribe(rb, &sink).await?;
        }
    }

    if let Some(lc) = &ctx.load_cell {
        lc.subscribe(&sink).await?;
        lc.set_bias().await?;
        lc.start_streaming().await?;
    }

    sink.clear_buffer().await;
    sink.set_time_now().await;
    sink.set_record(true);

    sleep(Duration::from_secs_f32(2.)).await;

    sink.set_record(false);

    if let Some(lc) = &ctx.load_cell {
        lc.stop_streaming().await?;
    }

    sink.complete().await.encode(&mut io::stdout())?;

    Ok(())
}

struct HardwareContext {
    pub motion_capture: Option<MotionCapture>,
    pub load_cell: Option<LoadCell>,
    pub wind_shape: Option<WindShape>,
    pub robot_arm: Option<RobotArm>,
}

impl HardwareContext {
    async fn create(config: &Config) -> Result<Self> {
        Ok(Self {
            motion_capture: match &config.hardware.motion_capture {
                Some(cfg) => Some(
                    MotionCapture::connect(cfg.ip, cfg.multicast_ip)
                        .await
                        .wrap_err("Failed to connect to motion capture")?,
                ),
                None => None,
            },

            load_cell: match &config.hardware.load_cell {
                Some(cfg) => Some(
                    LoadCell::connect(cfg.ip)
                        .await
                        .wrap_err("Failed to connect to load cell")?,
                ),
                None => None,
            },

            robot_arm: match &config.hardware.robot_arm {
                Some(cfg) => Some(
                    RobotArm::connect(cfg.ip, cfg.port)
                        .await
                        .wrap_err("Failed to connect to robot arm")?,
                ),
                None => None,
            },

            wind_shape: match &config.hardware.wind_shape {
                Some(wind_shape) => Some(
                    WindShape::connect(wind_shape.ip)
                        .await
                        .wrap_err("Failed to connect to wind shape")?,
                ),
                None => None,
            },
        })
    }
}
