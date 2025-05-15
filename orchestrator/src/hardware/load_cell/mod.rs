use std::{
    fmt, io, mem,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use async_trait::async_trait;
use bincode::{Decode, Encode, error::DecodeError};
use eyre::{Context, Result};
use nalgebra::{Rotation3, Vector3};
use reqwest::get;
use serde::{Deserialize, Serialize};
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle};

use crate::{
    config::LoadCellConfig,
    data::sink::{DataSinkBuilder, StreamWriter},
    defs::PoseEuler,
    misc::network_config,
};

use super::HardwareAgent;

const API_PATH: &str = "/netftapi2.xml";
const PORT: u16 = 49152;

const COMMAND_HEADER: u16 = 0x1234;
const BUFFER_SIZE: usize = 8192; // 8KB

const STANDARD_FORCE_UNIT: &str = "N";
const STANDARD_TORQUE_UNIT: &str = "Nm";
const NAME: &str = "load_cell";
const CHANNELS: [&str; 6] = ["fx", "fy", "fz", "tx", "ty", "tz"];

const DEVICE_CUTOFF_FREQ: [u32; 13] = [0, 838, 326, 152, 73, 35, 18, 8, 5, 1500, 2000, 2500, 3000];

type Data = [f32; CHANNELS.len()];

pub struct LoadCell {
    inner: Arc<Inner>,
    task: JoinHandle<Result<(), TaskError>>,
}

struct Inner {
    config: NetFtApi2,
    link: Link,
    shared: Mutex<Shared>,
}

#[derive(Default)]
struct Shared {
    stream: Option<StreamWriter>,
    transform: Option<LoadTransform>,
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

#[derive(Debug, Deserialize)]
struct NetFtApi2 {
    #[serde(rename = "scfgfu")]
    force_unit: String,
    #[serde(rename = "scfgtu")]
    torque_unit: String,
    #[serde(rename = "cfgcpf")]
    force_counts: f32,
    #[serde(rename = "cfgcpt")]
    torque_counts: f32,
    #[serde(rename = "setuserfilter")]
    low_pass_filter: u8,
    #[serde(rename = "comrdtrate")]
    output_rate: u32,
    #[serde(rename = "runrate")]
    internal_rate: u32,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum LoadCellInstruction {
    SetBias,
}

impl LoadCell {
    pub async fn connect_from_config(config: &LoadCellConfig) -> Result<Self> {
        Self::connect(config.ip).await
    }

    pub async fn connect(ip: IpAddr) -> Result<Self> {
        let config = Self::fetch_config(ip).await?;

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;

        socket.connect((ip, PORT)).await?;

        let inner = Arc::new(Inner {
            config,
            link: Link(socket),
            shared: Mutex::new(Shared::default()),
        });

        let task = tokio::spawn(Self::load_cell_task(inner.clone()));

        Ok(LoadCell { inner, task })
    }

    pub async fn execute(&self, instruction: LoadCellInstruction) -> Result<()> {
        match instruction {
            LoadCellInstruction::SetBias => self.set_bias().await,
        }
        .wrap_err("Load cell error")
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

    pub async fn set_tool_offset(&self, offset: PoseEuler) {
        let transform = LoadTransform::new(offset);
        self.inner.shared.lock().await.transform = Some(transform);
    }

    async fn fetch_config(ip: IpAddr) -> Result<NetFtApi2> {
        let xml_str = get(&format!("http://{ip}{API_PATH}")).await?.text().await?;

        let config: NetFtApi2 =
            quick_xml::de::from_str(&xml_str).wrap_err("Failed to parse XML config")?;

        if config.force_unit != STANDARD_FORCE_UNIT {
            tracing::warn!(
                "Non-standard force unit \"{}\" (expected {STANDARD_FORCE_UNIT})",
                config.force_unit
            );
        }

        if config.torque_unit != STANDARD_TORQUE_UNIT {
            tracing::warn!(
                "Non-standard torque unit \"{}\" (expected {STANDARD_TORQUE_UNIT})",
                config.torque_unit
            );
        }

        if config.output_rate < config.internal_rate {
            tracing::warn!(
                "Output rate {} Hz is less than internal rate ({} Hz)",
                config.output_rate,
                config.internal_rate
            );
        }

        if config.low_pass_filter != 0 {
            tracing::warn!(
                "Device low-pass filter is enabled ({} Hz)",
                DEVICE_CUTOFF_FREQ[config.low_pass_filter as usize]
            );
        }

        Ok(config)
    }

    #[tracing::instrument(skip(inner))]
    async fn load_cell_task(inner: Arc<Inner>) -> Result<(), TaskError> {
        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        loop {
            inner.link.0.recv_buf(&mut buf).await?;

            let lock = inner.shared.lock().await;

            if let Some(stream) = &lock.stream {
                let msg = Message::from_bytes(&buf)?;
                let mut load = msg.to_load(&inner.config);

                if let Some(transform) = &lock.transform {
                    load = transform.apply(&load);
                }

                stream.add(&load.as_array()).await;
            }

            buf.clear();
        }
    }
}

#[async_trait]
impl HardwareAgent for LoadCell {
    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        let name = NAME.to_owned();
        let channels = CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.register_stream(name, channels).await;

        self.inner.shared.lock().await.stream = Some(stream);
    }

    async fn start(&mut self) {
        let _ = self.start_streaming().await;
    }

    async fn stop(&mut self) {
        let _ = self.stop_streaming().await;
    }
}

impl fmt::Display for LoadCell {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{NAME}")
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
        bincode::encode_into_slice(self, &mut buf, network_config()).unwrap();
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

#[derive(Copy, Clone, Debug)]
struct Load {
    force: Vector3<f32>,
    moment: Vector3<f32>,
}

impl Message {
    fn from_bytes(buf: &[u8]) -> Result<Self, DecodeError> {
        Ok(bincode::decode_from_slice(buf, network_config())?.0)
    }

    fn to_load(&self, config: &NetFtApi2) -> Load {
        let [fx, fy, fz] = [self.fx, self.fy, self.fz].map(|x| x as f32 / config.force_counts);
        let [tx, ty, tz] = [self.tx, self.ty, self.tz].map(|x| x as f32 / config.torque_counts);

        Load {
            force: Vector3::new(fx, fy, fz),
            moment: Vector3::new(tx, ty, tz),
        }
    }
}

impl Load {
    fn as_array(&self) -> [f32; 6] {
        [
            self.force.x,
            self.force.y,
            self.force.z,
            self.moment.x,
            self.moment.y,
            self.moment.z,
        ]
    }
}

/* == Transformation == */

struct LoadTransform {
    rotation: Rotation3<f32>,
    translation: Vector3<f32>,
}

impl LoadTransform {
    pub fn new(offset: PoseEuler) -> Self {
        let translation = Vector3::new(offset.x, offset.y, offset.z);
        let rotation = Rotation3::from_euler_angles(offset.rx, offset.ry, offset.rz);

        Self {
            rotation,
            translation,
        }
    }

    pub fn apply(&self, load: &Load) -> Load {
        let force = self.rotation * load.force;
        let moment = self.rotation * load.moment + self.translation.cross(&force);
        Load { force, moment }
    }
}

/* == Utils == */

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
