use std::{
    collections::HashMap,
    net::{Ipv4Addr, UdpSocket},
};

use eyre::Result;
use serde::Serialize;

use crate::recording::RecordedSample;

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

    pub fn send(&self, sample: &RecordedSample) -> Result<()> {
        let buffer = serde_json::to_vec(&Message::from(sample))?;
        self.socket.send_to(&buffer, self.addr)?;
        Ok(())
    }
}

impl From<&RecordedSample<'_>> for Message {
    fn from(value: &RecordedSample) -> Self {
        Message {
            ts: value.timestamp_s(),

            fields: value
                .definition
                .qualified_channel_names()
                .zip(value.channel_data.iter().copied())
                .collect(),
        }
    }
}
