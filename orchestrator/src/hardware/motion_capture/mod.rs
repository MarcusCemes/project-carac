use std::{
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use bytes::Bytes;
use eyre::{bail, Result};
use protocol::RigidBodyData;
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle};

use crate::{
    config::MotionCaptureConfig,
    data::sink::{DataSink, StreamWriter},
};

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
    stream: StreamWriter,
}

#[derive(Debug)]
enum TaskError {
    MalformedPacket,
    SocketClosed,
}

impl MotionCapture {
    pub async fn connect_from_config(config: &MotionCaptureConfig) -> Result<Self> {
        Self::connect(config.ip, config.multicast_ip).await
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

    pub async fn subscribe(&self, name: &str, sink: &DataSink) -> Result<()> {
        let description_lock = self.inner.description.lock().await;

        let Some(rigid_body) = description_lock.get_rb_name(name) else {
            bail!("Rigid body {name} not found");
        };

        let mut subscription_lock = self.inner.subscriptions.lock().await;

        if subscription_lock.iter().any(|s| s.id == rigid_body.id) {
            tracing::warn!("Already subscribed to rigid body: {name}");
            return Ok(());
        }

        let name = format!("{NAME}/{name}");
        let channels = CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.add_stream(name, channels).await?;

        subscription_lock.push(Subscription::new(rigid_body.id, stream));

        Ok(())
    }

    // pub async fn unsubscribe(&self, name: &str, sink: &DataSink) -> bool {
    //     let description_lock = self.inner.description.lock().await;

    //     if let Some(rigid_body) = description_lock.get_rb_name(name) {
    //         let mut lock = self.inner.subscriptions.lock().await;

    //         if let Some(pos) = lock.iter().position(|s| s.id == rigid_body.id) {
    //             lock.swap_remove(pos);

    //             let stream_name = format!("{NAME}/{name}");
    //             sink.remove_stream(&stream_name).await;
    //             return true;
    //         }
    //     }

    //     false
    // }

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

            let frame = protocol::parse_message(&*buf)
                .and_then(|msg| protocol::parse_frame_data(msg.payload))
                .map_err(|_| TaskError::MalformedPacket)
                .inspect_err(|e| {
                    tracing::error!("Failed to parse frame data: {:?}", e);
                })?;

            {
                let subscriptions = inner.subscriptions.lock().await;

                for rb in frame.rigid_bodies {
                    if let Some(subscription) = subscriptions.iter().find(|s| s.id == rb.id) {
                        let data: Data = rb.into();
                        subscription.stream.add(&data).await;
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

impl Drop for MotionCapture {
    fn drop(&mut self) {
        tracing::warn!("Stopping motion capture task");
        self.task.abort();
    }
}

impl Subscription {
    pub fn new(id: i32, stream: StreamWriter) -> Self {
        Subscription { id, stream }
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
