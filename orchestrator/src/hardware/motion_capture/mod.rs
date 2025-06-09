use std::{
    fmt,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use async_trait::async_trait;
use bytes::Buf;
use eyre::{Result, bail};
use nalgebra::Vector3;
use nat_net::{Message, ModelDefinitions, Request, RigidBodyData, RigidBodyDescription};
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle, time::Instant};

use crate::{
    config::MotionCaptureConfig,
    data::sink::{DataSinkBuilder, StreamWriter},
    defs::Point,
    hardware::HardwareAgent,
};

use self::protocol::*;

mod nat_net;
pub mod protocol;

/* === Definitions === */

pub struct MotionCapture {
    inner: Arc<Inner>,
    task: JoinHandle<Result<()>>,
}

struct Inner {
    link: Link,
    shared: Mutex<Shared>,
}

struct Shared {
    definitions: ModelDefinitions,
    subscriptions: Vec<Subscription>,
}

pub struct Subscription {
    pub id: i32,
    pub name: String,
    pub stream: Option<StreamWriter>,
}

/* === Implementations === */

impl MotionCapture {
    pub const NAME: &str = "track";
    pub const CHANNELS: [&str; Point::WIDTH] = Point::CHANNELS;

    pub async fn try_new(ip: IpAddr, multicast_ip: Ipv4Addr) -> Result<Self> {
        let cmd_socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
        cmd_socket.connect((ip, Link::CMD_PORT)).await?;

        let link = Link::new(cmd_socket);

        Self::handshake(&link).await?;

        let definitions = Self::fetch_model_definitions(&link).await?;

        // Discover the correct network interface for multicast
        let local_ip = match link.socket.local_addr()?.ip() {
            IpAddr::V4(ipv4) => ipv4,
            IpAddr::V6(_) => bail!("IPv6 is not supported for multicast discovery"),
        };

        drop(link);

        tracing::debug!("Using local interface {local_ip} for multicast");
        let data_socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, Link::DATA_PORT)).await?;
        data_socket.join_multicast_v4(multicast_ip, local_ip)?;

        let link = Link::new(data_socket);

        let inner = Arc::new(Inner {
            link,
            shared: Mutex::new(Shared {
                definitions,
                subscriptions: Vec::new(),
            }),
        });

        let task = tokio::spawn(Self::receiver_task(inner.clone()));

        Ok(MotionCapture { inner, task })
    }

    pub async fn try_new_from_config(config: &MotionCaptureConfig) -> Result<Self> {
        let module = Self::try_new(config.ip, config.multicast_ip).await?;

        for body in &config.rigid_bodies {
            module.subscribe(body.clone()).await?;
        }

        Ok(module)
    }

    /* == Handshake == */

    async fn handshake(link: &Link) -> Result<()> {
        tracing::debug!("Handshaking with motion capture server...");

        let mut buf = Vec::with_capacity(Link::RX_BUFFER_SIZE);

        link.send_request(Request::Connect, &mut buf).await?;

        match link.receive_message(&mut buf).await {
            Ok(Message::ServerInfo(response)) => response.validate_version()?,
            Ok(_) => bail!("Handshake failed: unexpected message type"),
            Err(e) => bail!("Handshake failed: {}", e),
        };

        tracing::info!("Handshake successful");

        Ok(())
    }

    /* == Model definitions == */

    async fn fetch_model_definitions(link: &Link) -> Result<ModelDefinitions> {
        tracing::info!("Requesting model definitions...");

        let mut buf = Vec::with_capacity(Link::RX_BUFFER_SIZE);

        link.send_request(Request::ModelDefinitions, &mut buf)
            .await?;

        loop {
            match link.receive_message(&mut buf).await {
                Ok(Message::ModelDefinitions(definitions)) => return Ok(definitions),
                Ok(_) => continue,
                Err(e) => bail!("Failed to fetch model definitions: {e}"),
            };
        }
    }

    /* == Subscription == */

    pub async fn subscribe(&self, name: String) -> Result<()> {
        let mut lock = self.inner.shared.lock().await;

        let Some(rb) = lock.definitions.get_rigid_body_by_name(&name) else {
            bail!("Rigid body definition for '{name}' not found");
        };

        if lock.subscriptions.iter().any(|s| s.id == rb.id) {
            tracing::warn!("Already subscribed to rigid body: {name}");
            return Ok(());
        }

        tracing::debug!("Subscribing to rigid body: {name} (ID: {})", rb.id);

        let subscription = Subscription {
            id: rb.id,
            name,
            stream: None,
        };

        lock.subscriptions.push(subscription);

        Ok(())
    }

    /* == Background tasks == */

    async fn receiver_task(inner: Arc<Inner>) -> Result<()> {
        let mut buf = Vec::with_capacity(Link::RX_BUFFER_SIZE);

        loop {
            let message = inner.link.receive_message(&mut buf).await?;
            let now = Instant::now();

            let Message::DataFrame(data_frame) = message else {
                let id = (&buf[..]).try_get_u16_le().unwrap_or(0);
                tracing::warn!("Unexpected message ID: {id}");
                continue;
            };

            let lock = inner.shared.lock().await;

            for rigid_body in data_frame.rigid_bodies {
                if let Some(subscription) =
                    lock.subscriptions.iter().find(|s| s.id == rigid_body.id)
                {
                    if let Some(stream) = &subscription.stream {
                        let point = Point::from(&rigid_body);
                        stream.add(now, &point.array()).await;
                    }
                }
            }
        }
    }
}

#[async_trait]
impl HardwareAgent for MotionCapture {
    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        let mut lock = self.inner.shared.lock().await;

        for subscription in &mut lock.subscriptions {
            if subscription.stream.is_none() {
                let name = format!("{}/{}", Self::NAME, subscription.name);
                let channels = Self::CHANNELS.map(str::to_owned).to_vec();

                subscription.stream = Some(sink.register_stream(name, channels).await);
            }
        }
    }
}

impl fmt::Display for MotionCapture {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", Self::NAME)
    }
}

impl Drop for MotionCapture {
    fn drop(&mut self) {
        tracing::warn!("Stopping motion capture task for {}", Self::NAME);
        self.task.abort();
    }
}

impl ModelDefinitions {
    pub fn get_rigid_body_by_name(&self, name: &str) -> Option<&RigidBodyDescription> {
        self.rigid_bodies.iter().find(|rb| rb.name == name)
    }
}

impl From<&RigidBodyData> for Point {
    fn from(value: &RigidBodyData) -> Self {
        let (rx, ry, rz) = value.orientation.euler_angles();

        Point {
            position: value.position,
            orientation: Vector3::new(rx, ry, rz),
        }
    }
}
