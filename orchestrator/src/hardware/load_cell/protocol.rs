use std::{io, net::IpAddr};

use bytes::{Buf, BufMut};
use eyre::{Context, Result};
use nalgebra::Vector3;
use serde::Deserialize;
use tokio::net::UdpSocket;

use crate::{
    defs::Load,
    misc::buf::{Decode, DecodeError, Encode},
};

use super::LoadCell;

/* === Definitions === */

pub struct Link {
    socket: UdpSocket,
}

pub enum Instruction {
    SetBias,
    StartStreaming,
    StartBuffered,
    StopStreaming,
}

struct Request {
    instruction: Instruction,
    sample_count: u32,
}

pub struct Message {
    load: [i32; 6],
}

#[derive(Deserialize)]
pub struct NetFtApi2 {
    #[serde(rename = "scfgfu")]
    pub force_unit: String,
    #[serde(rename = "scfgtu")]
    pub torque_unit: String,
    #[serde(rename = "cfgcpf")]
    pub force_counts: u32,
    #[serde(rename = "cfgcpt")]
    pub torque_counts: u32,
    #[serde(rename = "setuserfilter")]
    pub low_pass_filter: u8,
    #[serde(rename = "comrdtrate")]
    pub output_rate: u32,
    #[serde(rename = "runrate")]
    pub internal_rate: u32,
    #[serde(rename = "cfgtfx")]
    pub tool_transform: [f32; 6],
    #[serde(rename = "comrdtbsiz")]
    pub buffered_size: u8,
}

/* === Implementations === */

impl Link {
    const COMMAND_HEADER: u16 = 0x1234;
    const REQUEST_SIZE: usize = 8;
    const MAX_BUFFER_SIZE: u8 = 40;

    const API_PATH: &str = "/netftapi2.xml";

    pub fn new(socket: UdpSocket) -> Self {
        Self { socket }
    }

    pub async fn receive_loads(
        &self,
        buf: &mut Vec<u8>,
        output: &mut Vec<f32>,
        config: &NetFtApi2,
    ) -> Result<usize> {
        buf.clear();
        output.clear();

        self.socket.recv_buf(buf).await?;

        let buf = &mut &buf[..];
        let mut count = 0;

        while buf.has_remaining() {
            let message = Message::decode(&mut &buf[..])?;
            let mut load = message.load(config.force_counts, config.torque_counts);

            Self::fix_orientation(&mut load);

            output.extend_from_slice(&load.array());
            count += 1;
        }

        Ok(count)
    }

    pub async fn send_instruction(&self, instruction: Instruction) -> io::Result<()> {
        let mut buf = Vec::with_capacity(Self::REQUEST_SIZE);

        Request::new(instruction).encode(&mut buf);

        self.socket.send(&buf).await?;

        Ok(())
    }

    pub async fn fetch_config(ip: IpAddr) -> Result<NetFtApi2> {
        let response = reqwest::get(&format!("http://{ip}{}", Self::API_PATH)).await?;
        let text = response.text().await?;

        let config: NetFtApi2 =
            quick_xml::de::from_str(&text).wrap_err("Failed to parse XML config")?;

        config.validate();

        Ok(config)
    }

    pub async fn set_variables<'a, I>(&self, page: &str, values: I) -> Result<()>
    where
        I: Iterator<Item = &'a (&'a str, &'a str)>,
    {
        let ip = self.socket.peer_addr()?.ip();
        let mut url = format!("http://{ip}/{page}.cgi?");

        for (i, (key, value)) in values.enumerate() {
            if i > 0 {
                url.push('&');
            }

            url.push_str(&format!("{key}={value}"));
        }

        reqwest::get(&url)
            .await
            .wrap_err("Failed to set variables")?;

        Ok(())
    }

    /// The datasheet specifies the X axis point towards the back of the device on the tool
    /// side. This function swaps the X and Y axes of the force and moment vectors.
    fn fix_orientation(load: &mut Load) {
        (load.force.x, load.force.y) = (load.force.y, -load.force.x);
        (load.moment.x, load.moment.y) = (load.moment.y, -load.moment.x);
    }
}

impl Request {
    pub fn new(instruction: Instruction) -> Self {
        Self {
            instruction,
            sample_count: 0,
        }
    }
}

impl Encode for Request {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        buf.put_u16(Link::COMMAND_HEADER);
        buf.put_u16(self.instruction.id());
        buf.put_u32(self.sample_count);
    }
}

impl Instruction {
    pub fn id(&self) -> u16 {
        match self {
            Self::StopStreaming => 0x00,
            Self::StartStreaming => 0x02,
            Self::StartBuffered => 0x03,
            Self::SetBias => 0x42,
        }
    }
}

impl Decode for Message {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let _rdt_sequence = buf.try_get_u32()?;
        let _ft_sequence = buf.try_get_u32()?;
        let _status = buf.try_get_u32()?;

        let mut load = [0; Load::WIDTH];

        for value in &mut load {
            *value = buf.try_get_i32()?;
        }

        Ok(Self { load })
    }
}

impl Message {
    pub fn load(&self, force_counts: u32, moment_counts: u32) -> Load {
        let [fx, fy, fz, mx, my, mz] = self.load.map(|v| v as f32);

        Load {
            force: Vector3::new(fx, fy, fz) / force_counts as f32,
            moment: Vector3::new(mx, my, mz) / moment_counts as f32,
        }
    }
}

impl NetFtApi2 {
    const CUTOFF_FREQ_MAP: [u32; 13] = [0, 838, 326, 152, 73, 35, 18, 8, 5, 1500, 2000, 2500, 3000];

    const TOOL_TRANSFORM: [f32; 6] = [0., 0., 0., 0., 0., 90.];

    fn validate(&self) {
        if self.force_unit != LoadCell::STANDARD_FORCE_UNIT {
            tracing::warn!(
                "Non-standard force unit \"{}\" (expected {})",
                self.force_unit,
                LoadCell::STANDARD_FORCE_UNIT
            );
        }

        if self.torque_unit != LoadCell::STANDARD_TORQUE_UNIT {
            tracing::warn!(
                "Non-standard torque unit \"{}\" (expected {})",
                self.torque_unit,
                LoadCell::STANDARD_TORQUE_UNIT
            );
        }

        if self.output_rate < self.internal_rate {
            tracing::warn!(
                "Output rate {} Hz is less than internal rate ({} Hz)",
                self.output_rate,
                self.internal_rate
            );
        }

        if self.low_pass_filter != 0 {
            tracing::warn!(
                "Device low-pass filter is enabled ({} Hz)",
                Self::CUTOFF_FREQ_MAP[self.low_pass_filter as usize]
            );
        }

        if self.tool_transform != Self::TOOL_TRANSFORM {
            tracing::warn!("Incorrect tool transform, this may lead to malformed data!");
            tracing::warn!(
                "Received {:?} (expected {:?})",
                self.tool_transform,
                Self::TOOL_TRANSFORM
            );
        } else {
            tracing::info!("Load transform set correctly");
        }

        tracing::info!(
            "RDT buffer set is set to {} samples (max {})",
            self.buffered_size,
            Link::MAX_BUFFER_SIZE
        );
    }
}
