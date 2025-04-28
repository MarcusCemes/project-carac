use std::{net::IpAddr, sync::Arc};

use eyre::Result;

use crate::hardware::{LoadCell, MotionCapture, RobotArm, WindShape};

#[derive(Default)]
pub struct Orchestrator(Arc<Inner>);

impl Orchestrator {
    pub fn new() -> Self {
        Orchestrator::default()
    }
}

#[derive(Default)]
struct Inner {}

/* == Hardware == */

#[derive(Default)]
pub struct Hardware {
    load_cell: Option<LoadCell>,
    motion_capture: Option<MotionCapture>,
    robot_arm: Option<RobotArm>,
    wind_shape: Option<WindShape>,
}

impl Hardware {
    pub async fn with_load_cell(mut self, ip: IpAddr) -> Result<Self> {
        let module = LoadCell::connect(ip).await?;
        self.load_cell = Some(module);
        Ok(self)
    }

    pub async fn with_motion_capture(
        mut self,
        ip: IpAddr,
        multicast_ip: std::net::Ipv4Addr,
    ) -> Result<Self> {
        let module = MotionCapture::connect(ip, multicast_ip).await?;
        self.motion_capture = Some(module);
        Ok(self)
    }

    pub async fn with_robot_arm(mut self, ip: IpAddr, port: u16) -> Result<Self> {
        let module = RobotArm::connect(ip, port).await?;
        self.robot_arm = Some(module);
        Ok(self)
    }
}
