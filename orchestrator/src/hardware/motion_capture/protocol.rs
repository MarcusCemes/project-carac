use std::io;

use eyre::{Context, Result};
use tokio::net::UdpSocket;

use crate::misc::buf::{Decode, Encode};

use super::nat_net::{Message, Request};

/* === Definitions === */

pub struct Link {
    socket: UdpSocket,
}

/* === Implementations === */

impl Link {
    pub const CMD_PORT: u16 = 1510;
    pub const DATA_PORT: u16 = 1511;

    pub const RX_BUFFER_SIZE: usize = 16384; // 16 KiB

    pub fn new(socket: UdpSocket) -> Self {
        Link { socket }
    }

    pub async fn send_request(&self, request: Request, buf: &mut Vec<u8>) -> io::Result<()> {
        buf.clear();
        request.encode(buf);

        self.socket.send(buf).await?;

        Ok(())
    }

    pub async fn receive_message(&self, buf: &mut Vec<u8>) -> Result<Message> {
        buf.clear();

        self.socket.recv_buf(buf).await?;

        Message::decode(&mut &buf[..]).wrap_err("Failed to decode message")
    }
}
