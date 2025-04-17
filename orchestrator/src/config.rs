use std::net::{IpAddr, Ipv4Addr};

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Config {
    pub motion_capture: Option<MotionCapture>,
    pub load_cell: Option<LoadCell>,
    pub robot_arm: Option<RobotArm>,
    pub wind_shape: Option<WindShape>,
    pub relay: Option<Relay>,
    pub devices: Option<Vec<Device>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct LoadCell {
    pub ip: IpAddr,
    #[serde(default)]
    pub filters: Filters,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MotionCapture {
    pub ip: IpAddr,
    #[serde(default = "default_multicast_addr")]
    pub multicast_ip: Ipv4Addr,
    pub rigid_bodies: Vec<String>,
    #[serde(default)]
    pub filters: Filters,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RobotArm {
    pub ip: IpAddr,
    pub port: u16,
    #[serde(default)]
    pub filters: Filters,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct WindShape {
    pub ip: IpAddr,
}

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Relay {
    pub plot_juggler: Option<PlotJuggler>,
}

#[derive(Debug, Serialize, Deserialize)]
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

impl Default for PlotJuggler {
    fn default() -> Self {
        PlotJuggler {
            ip: IpAddr::V4(Ipv4Addr::LOCALHOST),
            port: 9870,
        }
    }
}

impl Default for Filter {
    fn default() -> Self {
        Filter::Butterworth {
            cutoff: 0.,
            order: 0,
        }
    }
}

const fn default_multicast_addr() -> Ipv4Addr {
    Ipv4Addr::new(239, 255, 42, 99)
}
