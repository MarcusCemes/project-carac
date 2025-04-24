use std::{
    collections::HashMap,
    net::{IpAddr, Ipv4Addr, UdpSocket},
};

use serde::Serialize;

use crate::recording::RecordedSample;

const DEFAULT_IP: &str = "127.0.0.1";
const DEFAULT_PORT: u16 = 9870;

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

impl PlotJugglerBroadcaster {
    pub fn new(ip: Option<&str>) -> Self {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).expect("Failed to bind socket");
        let addr = (ip.unwrap_or(DEFAULT_IP).parse().unwrap(), DEFAULT_PORT);
        PlotJugglerBroadcaster { addr, socket }
    }

    pub fn send(&self, sample: &RecordedSample) {
        let ts = sample.timestamp_s();

        let fields = sample
            .definition
            .qualified_channel_names()
            .zip(sample.channel_data.iter().copied())
            .collect();

        let message = Message { ts, fields };

        let buffer = serde_json::to_vec(&message).expect("Failed to serialize message");

        self.socket
            .send_to(&buffer, self.addr)
            .expect("Failed to send message");
    }
}
