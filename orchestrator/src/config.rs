use std::{
    fmt::Display,
    net::{IpAddr, Ipv4Addr},
    path::PathBuf,
};

use eyre::{Context, Result};
use serde::{Deserialize, Serialize};
use tokio::fs::read;

use crate::{
    defs::Point,
    hardware::robot_arm::defs::Bounds as RobotArmBounds,
    misc::{ColourDot, serde::deserialize_null_to_default},
};

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct Config {
    pub hardware: HardwareConfig,
    pub sink: SinkConfig,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct HardwareConfig {
    pub additional_devices: Vec<DeviceConfig>,
    pub load_cell: Option<LoadCellConfig>,
    pub motion_capture: Option<MotionCaptureConfig>,
    pub robot_arm: Option<RobotArmConfig>,
    pub wind_shape: Option<WindShapeConfig>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoadCellConfig {
    pub ip: IpAddr,

    #[serde(default = "LoadCellConfig::update_settings")]
    pub configure_device: bool,

    #[serde(default = "LoadCellConfig::buffered_streaming")]
    pub buffered_streaming: bool,

    #[serde(default)]
    pub transform: Option<Point>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionCaptureConfig {
    pub ip: IpAddr,
    #[serde(default = "MotionCaptureConfig::multicast_addr")]
    pub multicast_ip: Ipv4Addr,

    pub rigid_bodies: Vec<String>,

    #[serde(default)]
    pub consider_as_skeleton: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotArmConfig {
    pub ip: IpAddr,

    #[serde(default = "RobotArmConfig::port")]
    pub port: u16,

    #[serde(default)]
    pub bounds: Option<RobotArmBounds>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WindShapeConfig {
    pub ip: IpAddr,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct SinkConfig {
    pub disable_audio: bool,
    pub mass: Option<f32>,
    pub session_path: Option<PathBuf>,

    #[serde(deserialize_with = "deserialize_null_to_default")]
    pub plot_juggler: Option<PlotJugglerConfig>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct PlotJugglerConfig {
    pub ip: IpAddr,
    pub port: u16,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeviceConfig {
    pub name: String,
    pub ip: IpAddr,
    pub port: u16,

    #[serde(default)]
    pub channels: Vec<String>,
    pub transmit_rate: u16,
}

/* == Default implementations == */

impl Default for PlotJugglerConfig {
    fn default() -> Self {
        PlotJugglerConfig {
            ip: IpAddr::V4(Ipv4Addr::LOCALHOST),
            port: 9870,
        }
    }
}

impl LoadCellConfig {
    const fn update_settings() -> bool {
        true
    }

    const fn buffered_streaming() -> bool {
        true
    }
}

impl MotionCaptureConfig {
    const fn multicast_addr() -> Ipv4Addr {
        Ipv4Addr::new(239, 255, 42, 99)
    }
}

impl RobotArmConfig {
    const fn port() -> u16 {
        20000
    }
}

/* == Parsing == */

impl Config {
    pub async fn load(path: &str) -> Result<Self> {
        let data = read(path).await.wrap_err("Failed to read config file")?;
        Self::parse(&data)
    }

    pub fn parse(data: &[u8]) -> Result<Self> {
        serde_yaml::from_slice(data).wrap_err("Failed to parse config")
    }
}

/* == Display == */

macro_rules! dot {
    ($option:expr) => {
        ColourDot::from(&$option)
    };
}

impl Display for HardwareConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = String::new();

        s.push_str("Hardware context:\n");
        s.push_str(&format!("{} Load cell\n", dot!(self.load_cell)));
        s.push_str(&format!("{} Motion capture\n", dot!(self.motion_capture)));
        s.push_str(&format!("{} Robot arm\n", dot!(self.robot_arm)));
        s.push_str(&format!("{} Wind shape\n", dot!(self.wind_shape)));

        for device in &self.additional_devices {
            s.push_str(&format!("{} {} (device)\n", ColourDot(true), device.name));
        }

        write!(f, "{s}")
    }
}
