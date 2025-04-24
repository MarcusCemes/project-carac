use std::net::{IpAddr, Ipv4Addr};

use serde::{Deserialize, Serialize};

use crate::misc::serde::deserialise_empty_to_default;

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Config {
    pub hardware: HardwareConfig,
    pub recording: RecordingConfig,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct HardwareConfig {
    pub additional_devices: Vec<Device>,
    pub load_cell: Option<LoadCellConfig>,
    pub motion_capture: Option<MotionCaptureConfig>,
    pub robot_arm: Option<RobotArmConfig>,
    pub wind_shape: Option<WindShapeConfig>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct LoadCellConfig {
    pub ip: IpAddr,

    #[serde(default)]
    pub filters: Filters,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MotionCaptureConfig {
    pub ip: IpAddr,
    #[serde(default = "default_multicast_addr")]
    pub multicast_ip: Ipv4Addr,

    pub rigid_bodies: Vec<String>,

    #[serde(default)]
    pub consider_as_skeleton: bool,

    #[serde(default)]
    pub filters: Filters,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RobotArmConfig {
    pub ip: IpAddr,
    pub port: u16,

    #[serde(default)]
    pub filters: Filters,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct WindShapeConfig {
    pub ip: IpAddr,
}

#[derive(Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct RecordingConfig {
    pub save_path: Option<String>,
    #[serde(deserialize_with = "deserialise_empty_to_default")]
    pub plot_juggler: Option<PlotJuggler>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct PlotJuggler {
    pub ip: IpAddr,
    pub port: u16,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Device {
    pub name: String,
    pub ip: IpAddr,
    pub port: u16,

    #[serde(default)]
    pub channels: Vec<String>,
    pub transmit_rate: u16,

    #[serde(default)]
    pub filters: Filters,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Filters(pub Vec<Filter>);

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum Filter {
    Butterworth { cutoff: f32, order: u32 },
}

/* == Default implementations == */

impl Default for PlotJuggler {
    fn default() -> Self {
        PlotJuggler {
            ip: IpAddr::V4(Ipv4Addr::LOCALHOST),
            port: 9870,
        }
    }
}

const fn default_multicast_addr() -> Ipv4Addr {
    Ipv4Addr::new(239, 255, 42, 99)
}
