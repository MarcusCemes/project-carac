use std::{io, mem, net::Ipv4Addr, sync::Arc};

use bincode::{config, error::DecodeError, Decode, Encode};
use reqwest::get;
use serde::Deserialize;
use tokio::{net::UdpSocket, task::JoinHandle};

use crate::recorder::RecordHandle;

const DIMENSIONS: usize = 6;

const DEFAULT_IP: &str = "192.168.1.1";
const PORT: u16 = 49152;

const COMMAND_HEADER: u16 = 0x1234;
const BUFFER_SIZE: usize = 8192; // 8KB
pub struct LoadCell {
    socket: Arc<UdpSocket>,
    task: JoinHandle<Result<(), TaskError>>,
}

#[derive(Debug)]
pub enum Error {
    ConfigFetch(reqwest::Error),
    Socket(io::Error),
    MalformedConfig(String),
}

#[derive(Debug)]
pub enum TaskError {
    MalformedPacket,
    SocketError(io::Error),
}

#[derive(Encode)]
struct Command {
    header: u16,
    command: u16,
    sample_count: u32,
}

#[derive(Deserialize)]
struct XmlConfig {
    #[serde(rename = "scalfu")]
    force_unit: String,
    #[serde(rename = "scaltu")]
    torque_unit: String,
    #[serde(rename = "calcpf")]
    force_counts: u32,
    #[serde(rename = "calcpt")]
    torque_counts: u32,
}

impl LoadCell {
    pub async fn connect(
        ip: Option<&str>,
        record: RecordHandle<DIMENSIONS>,
    ) -> Result<Self, Error> {
        let ip = ip.unwrap_or(DEFAULT_IP);
        let counts = Self::fetch_config(ip).await?;

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
        socket.connect((ip, PORT)).await?;

        let socket = Arc::new(socket);
        let task = tokio::spawn(Self::receiver_task(socket.clone(), record, counts));

        Ok(LoadCell { socket, task })
    }

    pub async fn set_streaming(&self, stream: bool, samples: Option<u32>) -> io::Result<()> {
        const BUFFER_SIZE: usize = mem::size_of::<Command>();

        let command = Command {
            header: COMMAND_HEADER,
            command: if stream { 0x2 } else { 0x0 },
            sample_count: samples.unwrap_or(0),
        };

        let mut buf = [0; BUFFER_SIZE];
        bincode::encode_into_slice(command, &mut buf, config()).unwrap();

        self.socket.send(&buf).await?;

        Ok(())
    }

    pub async fn set_bias(&self) -> io::Result<()> {
        const BUFFER_SIZE: usize = mem::size_of::<Command>();

        let command = Command {
            header: COMMAND_HEADER,
            command: 0x42,
            sample_count: 0,
        };

        let mut buf = [0; BUFFER_SIZE];
        bincode::encode_into_slice(command, &mut buf, config()).unwrap();

        self.socket.send(&buf).await?;

        Ok(())
    }

    async fn fetch_config(ip: &str) -> Result<(u32, u32), Error> {
        let xml_str = get(&format!("http://{ip}/netftcalapi.xml"))
            .await?
            .text()
            .await?;

        // Deserialize with serde and quick_xml
        let config: XmlConfig = quick_xml::de::from_str(&xml_str)
            .map_err(|_| Error::MalformedConfig("Failed to parse XML config".to_string()))?;

        if config.force_unit != "N" {
            tracing::warn!("Force unit is not N: {}", config.force_unit);
        }

        if config.torque_unit != "Nm" {
            tracing::warn!("Torque unit is not Nm: {}", config.torque_unit);
        }

        Ok((config.force_counts, config.torque_counts))
    }

    async fn receiver_task(
        socket: Arc<UdpSocket>,
        record: RecordHandle<DIMENSIONS>,
        counts: (u32, u32),
    ) -> Result<(), TaskError> {
        tracing::debug!("Starting receiver task");
        let (force_counts, torque_counts) = (counts.0 as f32, counts.1 as f32);

        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        loop {
            socket.recv_buf(&mut buf).await?;

            let msg = parse_message(&buf)?;

            let payload = [
                msg.fx as f32 / force_counts,
                msg.fy as f32 / force_counts,
                msg.fz as f32 / force_counts,
                msg.tx as f32 / torque_counts,
                msg.ty as f32 / torque_counts,
                msg.tz as f32 / torque_counts,
            ];

            record.append(payload).await;

            buf.clear();
        }
    }
}

impl Drop for LoadCell {
    fn drop(&mut self) {
        tracing::warn!("Stopping task");
        self.task.abort();
    }
}

/* == Message == */

#[derive(Decode)]
struct Message {
    _rdt_sequence: u32,
    _ft_sequence: u32,
    _status: u32,

    fx: i32,
    fy: i32,
    fz: i32,
    tx: i32,
    ty: i32,
    tz: i32,
}

fn parse_message(buf: &[u8]) -> Result<Message, DecodeError> {
    Ok(bincode::decode_from_slice(buf, config())?.0)
}

/* == Utils == */

fn config() -> impl config::Config {
    config::standard()
        .with_big_endian()
        .with_fixed_int_encoding()
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Error::Socket(err)
    }
}

impl From<reqwest::Error> for Error {
    fn from(err: reqwest::Error) -> Self {
        Error::ConfigFetch(err)
    }
}

impl From<DecodeError> for TaskError {
    fn from(_: DecodeError) -> Self {
        TaskError::MalformedPacket
    }
}

impl From<io::Error> for TaskError {
    fn from(err: io::Error) -> Self {
        TaskError::SocketError(err)
    }
}
