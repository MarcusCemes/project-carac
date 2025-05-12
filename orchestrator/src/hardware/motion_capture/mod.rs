use std::{
    fmt,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use async_trait::async_trait;
use bytes::Bytes;
use eyre::{Result, bail};
use protocol::RigidBodyData;
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle};

use crate::{
    config::MotionCaptureConfig,
    data::sink::{DataSinkBuilder, StreamWriter},
};

use super::HardwareAgent;

pub mod protocol;

const COMMAND_PORT: u16 = 1510;
const DATA_PORT: u16 = 1511;

const BUFFER_SIZE: usize = 16384; // 16 KB

const CHANNELS: [&str; 7] = ["x", "y", "z", "qx", "qy", "qz", "qw"];
const NAME: &str = "motion_capture";

type Data = [f32; CHANNELS.len()];

pub struct MotionCapture {
    inner: Arc<MotionCaptureInner>,
    task: JoinHandle<Result<(), TaskError>>,
}

struct MotionCaptureInner {
    description: Mutex<protocol::Description>,
    socket: UdpSocket,
    subscriptions: Mutex<Vec<Subscription>>,
}

struct Subscription {
    id: i32,
    name: String,
    stream: Option<StreamWriter>,
}

#[derive(Debug)]
enum TaskError {
    MalformedPacket,
    SocketClosed,
}

impl MotionCapture {
    pub async fn connect_from_config(config: &MotionCaptureConfig) -> Result<Self> {
        let module = Self::connect(config.ip, config.multicast_ip).await?;

        for body in &config.rigid_bodies {
            module.subscribe(body.clone()).await?;
        }

        Ok(module)
    }

    pub async fn connect(ip: IpAddr, multicast_ip: Ipv4Addr) -> Result<Self> {
        let description = {
            let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
            socket.connect((ip, COMMAND_PORT)).await?;

            Self::handshake(&socket).await?;
            Self::get_model_definitions(&socket).await?
        };

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, DATA_PORT)).await?;
        socket.join_multicast_v4(multicast_ip, Ipv4Addr::UNSPECIFIED)?;

        let inner = Arc::new(MotionCaptureInner {
            description: Mutex::new(description),
            socket,
            subscriptions: Default::default(),
        });

        let task = tokio::spawn(Self::receiver_task(inner.clone()));

        Ok(MotionCapture { inner, task })
    }

    pub async fn subscribe(&self, name: String) -> Result<()> {
        let lock = self.inner.description.lock().await;

        let Some(rigid_body) = lock.get_rb_name(&name) else {
            bail!("Rigid body {name} not found");
        };

        let mut lock = self.inner.subscriptions.lock().await;

        if lock.iter().any(|s| s.id == rigid_body.id) {
            tracing::warn!("Already subscribed to rigid body: {name}");
            return Ok(());
        }

        tracing::debug!("Subscribing to rigid body: {name}");

        lock.push(Subscription {
            id: rigid_body.id,
            name,
            stream: None,
        });

        Ok(())
    }

    pub async fn refresh_descriptions(&self) -> Result<()> {
        let new_description = Self::get_model_definitions(&self.inner.socket).await?;
        let mut description = self.inner.description.lock().await;

        *description = new_description;

        Ok(())
    }

    // async fn register_subscriptions(&self, sink: &Sink) -> Result<()> {
    //     let description = self.inner.description.lock().await;
    //     let mut subscriptions = self.inner.subscriptions.lock().await;

    //     for subscription in &mut *subscriptions {
    //         // Rigid body description must exist if a subscription is active
    //         let rb = description.get_rb(subscription.id).unwrap();

    //         if !sink.has_stream(&rb.name).await {
    //             let name = format!("{NAME}/{}", rb.name);
    //             let channels = CHANNELS.map(str::to_owned).to_vec();
    //             let stream = sink.add_stream(name, channels).await?;
    //             subscription.stream = Some(stream)
    //         }
    //     }

    //     Ok(())
    // }

    async fn receiver_task(inner: Arc<MotionCaptureInner>) -> Result<(), TaskError> {
        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        loop {
            inner
                .socket
                .recv_buf(&mut buf)
                .await
                .map_err(|_| TaskError::SocketClosed)?;

            let frame = protocol::parse_message(buf.as_ref())
                .and_then(|msg| protocol::parse_frame_data(msg.payload))
                .map_err(|_| TaskError::MalformedPacket)
                .inspect_err(|e| {
                    tracing::error!("Failed to parse frame data: {:?}", e);
                })?;

            {
                let subscriptions = inner.subscriptions.lock().await;

                for rb in frame.rigid_bodies {
                    if let Some(subscription) = subscriptions.iter().find(|s| s.id == rb.id) {
                        if let Some(stream) = &subscription.stream {
                            let data: Data = rb.into();
                            stream.add(&data).await;
                        }
                    }
                }
            }

            buf.clear();
        }
    }

    async fn handshake(socket: &UdpSocket) -> Result<()> {
        tracing::info!("Sending handshake message...");

        let msg = protocol::handshake_msg();
        socket.send(&msg.packet()).await?;

        let mut buf = Vec::with_capacity(BUFFER_SIZE);
        socket.recv_buf(&mut buf).await?;

        match protocol::parse_message(&mut Bytes::from(buf)) {
            Ok(msg) if msg.id == protocol::NAT_SERVERINFO => Ok(()),
            _ => bail!("Invalid handshake response"),
        }
    }

    async fn get_model_definitions(socket: &UdpSocket) -> Result<protocol::Description> {
        tracing::info!("Requesting model definitions...");

        let msg = protocol::request_definitions_msg();
        socket.send(&msg.packet()).await?;

        let mut buf = Vec::with_capacity(BUFFER_SIZE);
        socket.recv_buf(&mut buf).await?;

        tracing::info!("Received model definitions: {:?} B", buf.len());

        match protocol::parse_message(&mut Bytes::from(buf)) {
            Ok(msg) if msg.id == protocol::NAT_MODELDEF => {
                match protocol::parse_description(msg.payload) {
                    Ok(defs) => Ok(defs),

                    Err(_) => bail!("Invalid m odel definitions"),
                }
            }

            _ => bail!("Invalid handshake response"),
        }
    }
}

#[async_trait]
impl HardwareAgent for MotionCapture {
    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        for subscription in &mut *self.inner.subscriptions.lock().await {
            if subscription.stream.is_none() {
                let name = format!("{NAME}/{}", subscription.name);
                let channels = CHANNELS.map(str::to_owned).to_vec();
                subscription.stream = Some(sink.register_stream(name, channels).await);
            }
        }
    }
}

impl fmt::Display for MotionCapture {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{NAME}")
    }
}

impl Drop for MotionCapture {
    fn drop(&mut self) {
        tracing::warn!("Stopping motion capture task");
        self.task.abort();
    }
}

impl From<RigidBodyData> for Data {
    fn from(rb: RigidBodyData) -> Self {
        let RigidBodyData {
            position: (x, y, z),
            orientation: (qx, qy, qz, qw),
            ..
        } = rb;

        [x, y, z, qx, qy, qz, qw]
    }
}
