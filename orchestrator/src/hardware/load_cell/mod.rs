use std::{
    io, mem,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use bincode::{config, error::DecodeError, Decode, Encode};
use eyre::{Context, Result};
use reqwest::get;
use serde::Deserialize;
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle};

use crate::recording::{Sink, StreamWriter};

const DEFAULT_IP: &str = "192.168.1.1";
const PORT: u16 = 49152;

const COMMAND_HEADER: u16 = 0x1234;
const BUFFER_SIZE: usize = 8192; // 8KB

const STREAM_NAME: &str = "load_cell";
const CHANNELS: [&str; 6] = ["fx", "fy", "fz", "tx", "ty", "tz"];

type Data = [f32; CHANNELS.len()];
pub struct LoadCell {
    inner: Arc<LoadCellInner>,
    task: JoinHandle<Result<(), TaskError>>,
}

struct LoadCellInner {
    config: LoadCellConfig,
    link: Link,
    stream: Mutex<Option<StreamWriter>>,
}

struct Link(UdpSocket);

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
struct LoadCellConfig {
    #[serde(rename = "scalfu")]
    force_unit: String,
    #[serde(rename = "scaltu")]
    torque_unit: String,
    #[serde(rename = "calcpf")]
    force_counts: f32,
    #[serde(rename = "calcpt")]
    torque_counts: f32,

    #[serde(rename = "comrdtbsiz")]
    buffer_size: u8,
    #[serde(rename = "setuserfilter")]
    low_pass_frequency: u8,
    #[serde(rename = "comrdtrate")]
    output_rate: u32,
    #[serde(rename = "runrate")]
    internal_rate: u32,
}

impl LoadCell {
    pub async fn connect(ip: IpAddr) -> Result<Self> {
        let config = Self::fetch_config(ip).await?;

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
        socket.connect((ip, PORT)).await?;

        let inner = Arc::new(LoadCellInner {
            config,
            link: Link(socket),
            stream: Mutex::new(None),
        });

        let task = tokio::spawn(Self::load_cell_task(inner.clone()));

        Ok(LoadCell { inner, task })
    }

    pub async fn subscribe(&self, sink: &Sink) -> Result<()> {
        let channels = CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.add_stream(STREAM_NAME.to_owned(), channels).await?;

        *self.inner.stream.lock().await = Some(stream);

        Ok(())
    }

    pub async fn start_streaming(&self) -> io::Result<()> {
        self.inner.link.send(Command::new(0x2)).await
    }

    pub async fn stop_streaming(&self) -> io::Result<()> {
        self.inner.link.send(Command::new(0x0)).await
    }

    pub async fn set_bias(&self) -> io::Result<()> {
        self.inner.link.send(Command::new(0x42)).await
    }

    async fn fetch_config(ip: IpAddr) -> Result<LoadCellConfig> {
        let xml_str = get(&format!("http://{ip}/netftcalapi.xml"))
            .await?
            .text()
            .await?;

        // Deserialize with serde and quick_xml
        let config: LoadCellConfig =
            quick_xml::de::from_str(&xml_str).wrap_err("Failed to parse XML config")?;

        if config.force_unit != "N" {
            tracing::warn!("Force unit is not N: {}", config.force_unit);
        }

        if config.torque_unit != "Nm" {
            tracing::warn!("Torque unit is not Nm: {}", config.torque_unit);
        }

        Ok(config)
    }

    #[tracing::instrument(skip(inner))]
    async fn load_cell_task(inner: Arc<LoadCellInner>) -> Result<(), TaskError> {
        tracing::debug!("Receiver started");

        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        loop {
            inner.link.0.recv_buf(&mut buf).await?;

            if let Some(ref stream) = *inner.stream.lock().await {
                let msg = parse_message(&buf)?;
                let data = msg.data(&inner.config);

                stream.write_now(&data).await;
            }

            buf.clear();
        }
    }
}

impl Drop for LoadCell {
    fn drop(&mut self) {
        let inner = self.inner.clone();

        tokio::spawn(async move {
            let command = Command::new(0x0);
            inner.link.send(command).await
        });

        self.task.abort();
    }
}

impl Link {
    async fn send(&self, command: Command) -> io::Result<()> {
        self.0.send(&command.encode()).await?;
        Ok(())
    }
}

impl Command {
    fn new(command: u16) -> Self {
        Self {
            header: COMMAND_HEADER,
            command,
            sample_count: 0,
        }
    }

    fn with_samples(mut self, sample_count: u32) -> Self {
        self.sample_count = sample_count;
        self
    }

    fn encode(&self) -> [u8; 8] {
        let mut buf = [0; mem::size_of::<Command>()];
        bincode::encode_into_slice(self, &mut buf, binary_config()).unwrap();
        buf
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

impl Message {
    fn data(&self, config: &LoadCellConfig) -> Data {
        let [fx, fy, fz] = [self.fx, self.fy, self.fz].map(|x| x as f32 / config.force_counts);
        let [tx, ty, tz] = [self.tx, self.ty, self.tz].map(|x| x as f32 / config.torque_counts);
        [fx, fy, fz, tx, ty, tz]
    }
}

fn parse_message(buf: &[u8]) -> Result<Message, DecodeError> {
    Ok(bincode::decode_from_slice(buf, binary_config())?.0)
}

/* == Utils == */

fn binary_config() -> impl config::Config {
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
