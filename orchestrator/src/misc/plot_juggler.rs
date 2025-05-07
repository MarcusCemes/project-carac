use std::{
    collections::HashMap,
    net::{Ipv4Addr, UdpSocket},
};

use eyre::Result;
use serde::Serialize;

use crate::data::run::{RunSample, StreamInfo};

const DEFAULT_PORT: u16 = 9870;

/* == PlotJugglerBroadcaster == */

pub struct PlotJugglerBroadcaster {
    addr: (Ipv4Addr, u16),
    socket: UdpSocket,
}

#[derive(Debug, Serialize)]
struct Message {
    ts: f32,
    #[serde(flatten)]
    fields: HashMap<String, f32>,
}

impl PlotJugglerBroadcaster {
    pub fn create(ip: Option<&str>, port: Option<u16>) -> Result<Self> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;

        let ip = match ip {
            Some(ip) => ip.parse()?,
            None => Ipv4Addr::LOCALHOST,
        };

        let addr = (ip, port.unwrap_or(DEFAULT_PORT));

        Ok(PlotJugglerBroadcaster { addr, socket })
    }

    pub fn send(&self, sample: &RunSample, streams: &[StreamInfo]) -> Result<()> {
        let message = Message::new(sample, streams);
        let buffer = serde_json::to_vec(&message)?;

        self.socket.send_to(&buffer, self.addr)?;

        Ok(())
    }
}

impl Message {
    fn new(sample: &RunSample, streams: &[StreamInfo]) -> Self {
        Message {
            ts: sample.time_s(),

            fields: streams
                .iter()
                .flat_map(StreamInfo::qualified_channel_names)
                .zip(sample.channel_data.iter().copied())
                .collect(),
        }
    }
}
