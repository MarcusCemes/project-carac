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
    counts: LoadCounts,
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
    pub force_counts: f32,
    #[serde(rename = "cfgcpt")]
    pub torque_counts: f32,
    #[serde(rename = "setuserfilter")]
    pub low_pass_filter: u8,
    #[serde(rename = "comrdtrate")]
    pub output_rate: u32,
    #[serde(rename = "runrate")]
    pub internal_rate: u32,
    #[serde(rename = "cfgtfx")]
    pub tool_transform: String,
    #[serde(rename = "comrdtbsiz")]
    pub buffered_size: u8,
}

pub struct LoadCounts {
    force: f32,
    torque: f32,
}

impl From<&NetFtApi2> for LoadCounts {
    fn from(config: &NetFtApi2) -> Self {
        Self {
            force: config.force_counts,
            torque: config.torque_counts,
        }
    }
}

/* === Implementations === */

impl Link {
    pub const PORT: u16 = 49152;
    pub const BUFFER_SIZE: usize = 4096;

    const COMMAND_HEADER: u16 = 0x1234;
    const REQUEST_SIZE: usize = 8;
    const MAX_LOADS_PER_PACKET: u8 = 40;

    const API_PATH: &str = "/netftapi2.xml";

    pub fn new(counts: LoadCounts, socket: UdpSocket) -> Self {
        Self { counts, socket }
    }

    pub async fn receive_loads<'a>(
        &'a self,
        buf: &'a mut Vec<u8>,
    ) -> Result<impl ExactSizeIterator<Item = Load> + 'a> {
        buf.clear();

        self.socket.recv_buf(buf).await?;

        // Ensure the buffer is the correct multiple of message bytes
        debug_assert_eq!(buf.len() % Message::SIZE_B, 0,);

        // Number of messages in the buffer
        let count = buf.len() / Message::SIZE_B;

        // Iterate over each 6-f32 window and decode a load
        Ok((0..count).map(move |i| {
            let start = i * Message::SIZE_B;
            let end = start + Message::SIZE_B;

            Message::decode(&mut &buf[start..end])
                .expect("Malformed load cell message")
                .load(&self.counts)
        }))
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

impl Message {
    pub const SIZE_B: usize = 36; // u32 (3x) + i32 (6x)
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
    pub fn load(&self, load_counts: &LoadCounts) -> Load {
        let [fx, fy, fz, mx, my, mz] = self.load.map(|v| v as f32);

        Load {
            force: Vector3::new(fx, fy, fz) / load_counts.force,
            moment: Vector3::new(mx, my, mz) / load_counts.torque,
        }
    }
}

impl NetFtApi2 {
    const CUTOFF_FREQ_MAP: [u32; 13] = [0, 838, 326, 152, 73, 35, 18, 8, 5, 1500, 2000, 2500, 3000];

    const TOOL_TRANSFORM: &str = "0;0;0;0;0;0";

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
            tracing::warn!("Tool transform enabled! This may cause unexpected behavior.");
        }

        tracing::info!(
            "RDT buffer set is set to {} samples (max {})",
            self.buffered_size,
            Link::MAX_LOADS_PER_PACKET
        );
    }
}
