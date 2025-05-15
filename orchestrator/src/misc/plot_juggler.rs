use std::{
    collections::HashMap,
    net::{IpAddr, Ipv4Addr, UdpSocket},
};

use eyre::Result;
use serde::Serialize;

use crate::{config::PlotJugglerConfig, data::sink::StreamInfo};

const DEFAULT_PORT: u16 = 9870;

/* == PlotJugglerBroadcaster == */

pub struct PlotJugglerBroadcasterBuilder {
    ip: IpAddr,
    port: u16,
}

pub struct PlotJugglerBroadcaster {
    addr: (IpAddr, u16),
    socket: UdpSocket,
}

#[derive(Debug, Serialize)]
struct Message {
    ts: f32,

    #[serde(flatten)]
    fields: HashMap<String, f32>,
}

impl PlotJugglerBroadcasterBuilder {
    pub fn with_config(mut self, config: &PlotJugglerConfig) -> Self {
        self.ip = config.ip;
        self.port = config.port;

        self
    }

    pub fn build(self) -> Result<PlotJugglerBroadcaster> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
        let addr = (self.ip, self.port);

        Ok(PlotJugglerBroadcaster { addr, socket })
    }
}

impl Default for PlotJugglerBroadcasterBuilder {
    fn default() -> Self {
        Self {
            ip: PlotJugglerBroadcaster::DEFAULT_IP,
            port: PlotJugglerBroadcaster::DEFAULT_PORT,
        }
    }
}

impl PlotJugglerBroadcaster {
    const DEFAULT_IP: IpAddr = IpAddr::V4(Ipv4Addr::LOCALHOST);
    const DEFAULT_PORT: u16 = 9870;

    pub fn builder() -> PlotJugglerBroadcasterBuilder {
        PlotJugglerBroadcasterBuilder::default()
    }

    pub fn send(&self, ts: f32, channel_data: &[f32], streams: &StreamInfo) -> Result<()> {
        let buffer = serde_json::to_vec(&Message {
            ts,
            fields: fields(channel_data, streams),
        })?;

        self.socket.send_to(&buffer, self.addr)?;

        Ok(())
    }
}

fn fields(channel_data: &[f32], stream: &StreamInfo) -> HashMap<String, f32> {
    stream
        .qualified_channel_names()
        .into_iter()
        .zip(channel_data.iter().copied())
        .collect()
}
