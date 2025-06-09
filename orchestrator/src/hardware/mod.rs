use std::{fmt::Display, future::Future, time::Duration};

use async_trait::async_trait;
use eyre::{Context, Report, Result, eyre};
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

use crate::{
    HARDWARE_TIMEOUT, config::HardwareConfig, data::sink::DataSinkBuilder, misc::type_name,
};

#[async_trait]
pub trait HardwareAgent: Display + Send {
    async fn register(&mut self, sink: &mut DataSinkBuilder);

    async fn error(&mut self) -> Result<(), Report> {
        Ok(())
    }

    async fn clear_error(&mut self) {}
    async fn bias(&mut self) {}
    async fn start(&mut self) {}
    async fn stop(&mut self) {}
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

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut dyn HardwareAgent> {
        dynamic(&mut self.additional_devices)
            .chain(dynamic(&mut self.motion_capture))
            .chain(dynamic(&mut self.load_cell))
            .chain(dynamic(&mut self.robot_arm))
            .chain(dynamic(&mut self.wind_shape))
    }
}

/* == HardwareContextBuilder == */

pub struct HardwareContextBuilder {
    timeout: Option<Duration>,
}

impl HardwareContextBuilder {
    pub fn with_timeout(mut self, timeout: Option<Duration>) -> Self {
        self.timeout = timeout;
        self
    }

    pub async fn build(self, config: &HardwareConfig) -> Result<HardwareContext> {
        let initialiser = HardwareInitializer::new(self.timeout);

        let motion_capture = initialiser
            .init(&config.motion_capture, MotionCapture::try_new_from_config)
            .await?;

        let load_cell = initialiser
            .init(&config.load_cell, LoadCell::try_new_from_config)
            .await?;

        let robot_arm = initialiser
            .init(&config.robot_arm, RobotArm::try_new_from_config)
            .await?;

        let wind_shape = initialiser
            .init(&config.wind_shape, WindShape::connect_from_config)
            .await?;

        let mut additional_devices = Vec::with_capacity(config.additional_devices.len());

        for device in &config.additional_devices {
            let device = initialiser
                .init(&Some(device), |c| {
                    additional_device::Device::connect_from_config(c)
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

impl Default for HardwareContextBuilder {
    fn default() -> Self {
        Self {
            timeout: Some(HARDWARE_TIMEOUT),
        }
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

/// Helpful generic function that converts an iterable object of concrete HardwareAgent type (such as
/// an Option or a Vec) and maps them to a trait object for dynamic dispatch using a v-table.
fn dynamic<'a, T, U>(device: T) -> impl Iterator<Item = &'a mut dyn HardwareAgent>
where
    T: IntoIterator<Item = &'a mut U>,
    U: HardwareAgent + 'a,
{
    device.into_iter().map(|d| d as &mut dyn HardwareAgent)
}
