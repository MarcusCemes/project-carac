use std::{future::Future, time::Duration};

use eyre::{eyre, Context, Report, Result};
use load_cell::LoadCell;
use motion_capture::MotionCapture;
use robot_arm::RobotArm;
use wind_shape::WindShape;

pub mod additional_device;
pub mod example_counter;
pub mod load_cell;
pub mod motion_capture;
pub mod robot_arm;
pub mod wind_shape;

use crate::{config::HardwareConfig, misc::type_name};

trait Hardware {
    async fn errored(&mut self) -> Option<Report> {
        None
    }

    async fn on_pause(&mut self) {}
    async fn on_record(&mut self) {}
    async fn reset(&mut self) {}
}

/* == HardwareContext == */

pub struct HardwareContext {
    pub additional_devices: Vec<additional_device::Device>,
    pub motion_capture: Option<motion_capture::MotionCapture>,
    pub load_cell: Option<load_cell::LoadCell>,
    pub wind_shape: Option<wind_shape::WindShape>,
    pub robot_arm: Option<robot_arm::RobotArm>,
}

impl HardwareContext {
    pub fn builder() -> HardwareContextBuilder {
        HardwareContextBuilder::default()
    }
}

/* == HardwareContextBuilder == */

#[derive(Default)]
pub struct HardwareContextBuilder {
    timeout: Option<Duration>,
}

impl HardwareContextBuilder {
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    pub async fn build(self, config: &HardwareConfig) -> Result<HardwareContext> {
        let fuse = HardwareInitializer::new(self.timeout);

        let motion_capture = fuse
            .init(&config.motion_capture, MotionCapture::connect_from_config)
            .await?;

        let load_cell = fuse
            .init(&config.load_cell, LoadCell::connect_from_config)
            .await?;

        let robot_arm = fuse
            .init(&config.robot_arm, RobotArm::connect_from_config)
            .await?;

        let wind_shape = fuse
            .init(&config.wind_shape, WindShape::connect_from_config)
            .await?;

        let mut additional_devices = Vec::with_capacity(config.additional_devices.len());

        for device in &config.additional_devices {
            let device = fuse
                .init(&Some(device), |c| {
                    additional_device::Device::connect_from_config(*c)
                })
                .await?
                .unwrap();

            additional_devices.push(device);
        }

        Ok(HardwareContext {
            additional_devices,
            load_cell,
            motion_capture,
            robot_arm,
            wind_shape,
        })
    }
}

struct HardwareInitializer {
    timeout: Option<Duration>,
}

impl HardwareInitializer {
    fn new(timeout: Option<Duration>) -> Self {
        Self { timeout }
    }

    async fn init<'a, C, F, O, T>(&self, config: &'a Option<C>, factory: F) -> Result<Option<T>>
    where
        F: Fn(&'a C) -> O,
        O: Future<Output = Result<T>>,
    {
        Ok(match config {
            Some(config) => Some(match self.timeout {
                Some(duration) => tokio::time::timeout(duration, factory(config))
                    .await
                    .map_err(|_| eyre!("Connection timeout"))
                    .wrap_err_with(|| eyre!("Failed to initialise {}", type_name::<T>()))??,

                None => factory(config).await?,
            }),

            None => None,
        })
    }
}
