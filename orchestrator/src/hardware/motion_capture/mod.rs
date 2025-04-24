use std::{
    io,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use bytes::Bytes;
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle};

use crate::recording::{Recorder, StreamHandle};

pub mod protocol;

const COMMAND_PORT: u16 = 1510;
const DATA_PORT: u16 = 1511;

const BUFFER_SIZE: usize = 16384; // 8KB

const CHANNELS: [&str; 7] = ["x", "y", "z", "qx", "qy", "qz", "qw"];
const STREAM_NAME: &str = "motion_capture";

pub struct MotionCapture {
    inner: Arc<MotionCaptureInner>,
    task: JoinHandle<Result<(), TaskError>>,
}

struct MotionCaptureInner {
    description: Mutex<protocol::Description>,
    socket: UdpSocket,
    subscriptions: Mutex<Vec<(i32, StreamHandle)>>,
}

#[derive(Debug)]
enum TaskError {
    MalformedPacket,
    SocketClosed,
}

impl MotionCapture {
    pub async fn connect(ip: IpAddr, multicast_ip: Ipv4Addr) -> io::Result<MotionCapture> {
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

    pub async fn subscribe<S>(&self, name: S, recorder: &Recorder)
    where
        S: AsRef<str> + Into<String>,
    {
        let description_lock = self.inner.description.lock().await;

        let rigid_body = description_lock
            .get_rb(name.as_ref())
            .expect("Rigid body not found");

        let mut subscription_lock = self.inner.subscriptions.lock().await;

        if subscription_lock.iter().any(|(id, _)| *id == rigid_body.id) {
            tracing::warn!("Already subscribed to rigid body: {}", name.as_ref());
            return;
        }

        if !subscription_lock.iter().any(|s| s.0 == rigid_body.id) {
            let stream_name = format!("{STREAM_NAME}/{}", name.as_ref());
            let stream = recorder.add_stream(stream_name, &CHANNELS).await;
            subscription_lock.push((rigid_body.id, stream));
        }
    }

    pub async fn unsubscribe(&self, name: &str) -> bool {
        let description_lock = self.inner.description.lock().await;

        if let Some(rigid_body) = description_lock.get_rb(name) {
            let mut lock = self.inner.subscriptions.lock().await;

            if let Some(pos) = lock.iter().position(|s| s.0 == rigid_body.id) {
                lock.swap_remove(pos);
                return true;
            }
        }

        false
    }

    pub async fn refresh_descriptions(&self) -> io::Result<()> {
        let new_description = Self::get_model_definitions(&self.inner.socket).await?;
        let mut description = self.inner.description.lock().await;

        *description = new_description;

        Ok(())
    }

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
                    if let Some((_, stream)) = subscriptions.iter().find(|s| s.0 == rb.id) {
                        let p = &rb.position;
                        let q = &rb.orientation;

                        stream.add(&[p.0, p.1, p.2, q.0, q.1, q.2, q.3]).await;
                    }
                }
            }

            buf.clear();
        }
    }

    async fn handshake(socket: &UdpSocket) -> io::Result<()> {
        tracing::info!("Sending handshake message...");

        let msg = protocol::handshake_msg();
        socket.send(&msg.packet()).await?;

        let mut buf = Vec::with_capacity(BUFFER_SIZE);
        socket.recv_buf(&mut buf).await?;

        match protocol::parse_message(&mut Bytes::from(buf)) {
            Ok(msg) if msg.id == protocol::NAT_SERVERINFO => Ok(()),

            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Invalid handshake response",
            )),
        }
    }

    async fn get_model_definitions(socket: &UdpSocket) -> io::Result<protocol::Description> {
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

                    Err(_) => Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "Invalid model definitions",
                    )),
                }
            }

            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Invalid handshake response",
            )),
        }
    }
}

impl Drop for MotionCapture {
    fn drop(&mut self) {
        tracing::warn!("Stopping motion capture task");
        self.task.abort();
    }
}
