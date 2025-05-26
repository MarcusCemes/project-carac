use std::{
    io::Write,
    net::{IpAddr, Ipv4Addr},
};

use bytes::BufMut;
use eyre::{Result, eyre};
use tokio::net::UdpSocket;

use crate::misc::buf::Encode;

use super::{WindShape, defs::Status};

/* === Definitions === */

pub struct Link {
    socket: UdpSocket,
}

#[derive(Debug)]
pub struct Request {
    client_id: u8,
    instruction: Instruction,
}

#[derive(Debug)]
pub enum Instruction {
    InitiateConnection,
    Module { powered: bool, fan_speed: u8 },
    Status { request_control: bool },
}

#[derive(Debug)]
pub struct Response {
    pub client_id: u8,
    pub payload: ResponsePayload,
}

#[derive(Debug)]
pub enum ResponsePayload {
    Address,
    Module,
    Status(Status),
}

/* === Implementations === */

impl Link {
    pub const RX_BUFFER_SIZE: usize = 65536;
    pub const TX_BUFFER_SIZE: usize = 16384;

    const HANDSHAKE_CLIENT_ID: u8 = 0;

    pub async fn try_new(ip: IpAddr, remote_port: u16, local_port: u16) -> Result<Self> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, local_port)).await?;

        socket.connect((ip, remote_port)).await?;

        Ok(Self { socket })
    }

    pub async fn send_request(&self, request: Request) -> Result<()> {
        let mut buf = Vec::with_capacity(Self::TX_BUFFER_SIZE);

        request.encode(&mut buf);
        self.socket.send(&buf).await?;

        Ok(())
    }

    pub async fn receive_response(&self, buf: &mut Vec<u8>) -> Result<Response> {
        buf.clear();

        self.socket.recv_buf_from(buf).await?;

        Response::parse(buf).ok_or_else(|| eyre!("Failed to parse response"))
    }

    /* == Handshake == */

    pub async fn handshake(&self) -> Result<u8> {
        tracing::debug!("Handshaking with WindShape...");

        let request = Request::new(Self::HANDSHAKE_CLIENT_ID, Instruction::InitiateConnection);
        self.send_request(request).await?;

        loop {
            let mut buf = Vec::with_capacity(Self::RX_BUFFER_SIZE);
            let response = self.receive_response(&mut buf).await?;

            if let ResponsePayload::Address = response.payload {
                tracing::info!("Handshake successful");
                return Ok(response.client_id);
            }
        }
    }
}

impl Request {
    pub fn new(client_id: u8, instruction: Instruction) -> Self {
        Self {
            client_id,
            instruction,
        }
    }
}

impl Encode for Request {
    fn encode<B: bytes::BufMut>(&self, buf: &mut B) {
        let tag = self.instruction.tag();
        let _ = write!(buf.writer(), "{}@{}:", tag, self.client_id);

        self.instruction.encode_payload(buf);

        let _ = write!(buf.writer(), "\0");
    }
}

impl Instruction {
    pub fn tag(&self) -> &'static str {
        match self {
            Instruction::InitiateConnection => "REQUEST_CONNECTION",
            Instruction::Module { .. } => "MODULE",
            Instruction::Status { .. } => "STATUS",
        }
    }

    pub fn encode_payload<B: BufMut>(&self, buf: &mut B) {
        match self {
            Instruction::InitiateConnection => buf.put_slice(b"no_message"),

            Instruction::Module { powered, fan_speed } => {
                let powered = if *powered { 1 } else { 0 };
                let fan_speed = format!("{}", (*fan_speed).min(100));

                for i in 0..WindShape::MODULE_COUNT {
                    if i > 0 {
                        buf.put_u8(b'|');
                    }

                    let _ = write!(buf.writer(), "{};", i + 1);

                    for j in 0..WindShape::MODULE_FANS {
                        if j > 0 {
                            buf.put_u8(b',');
                        }

                        buf.put_slice(fan_speed.as_bytes());
                    }

                    let _ = write!(buf.writer(), ";{};0;0;0", powered);
                }
            }

            Instruction::Status { request_control } => {
                buf.put_u8(if *request_control { b'1' } else { b'0' });
                buf.put_slice(b";0");
            }
        }
    }
}

impl Response {
    pub fn parse(buf: &[u8]) -> Option<Self> {
        let input = std::str::from_utf8(buf).ok()?;

        let (header, payload) = input.split_once(':')?;
        let (tag, client_id_str) = header.split_once('@')?;
        let client_id = client_id_str.parse().ok()?;

        let payload = match tag {
            // We don't really care much for these payloads
            "ADDRESS" => ResponsePayload::Address,
            "MODULE" => ResponsePayload::Module,

            "STATUS" => {
                let in_control_str = payload.split(';').next()?;
                let in_control = in_control_str.parse::<u8>().ok()? == 1;

                ResponsePayload::Status(Status { in_control })
            }

            _ => return None,
        };

        Some(Response { client_id, payload })
    }
}
