use std::{io, net::Ipv4Addr, str::FromStr, sync::Arc};

use bytes::{Bytes, BytesMut};
use tokio::{net::UdpSocket, task::JoinHandle};

use crate::recorder::RecordHandle;

pub mod protocol;

const DIMENSIONS: usize = 7;

const DEFAULT_IP: &str = "192.168.100.184";
const DEFAULT_MULTICAST_IP: &str = "239.255.42.99";
const COMMAND_PORT: u16 = 1510;
const DATA_PORT: u16 = 1511;

const DEFAULT_BUFFER_CAPACITY: usize = 8192; // 8KB

pub struct MotionCapture {
    task: JoinHandle<Result<(), TaskError>>,
}

enum TaskError {
    MalformedPacket,
    SocketClosed,
}

impl MotionCapture {
    pub async fn create(
        ip: Option<&str>,
        record: RecordHandle<DIMENSIONS>,
    ) -> io::Result<MotionCapture> {
        let description = {
            let ip = ip.unwrap_or(DEFAULT_IP);

            let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
            socket.connect((ip, COMMAND_PORT)).await?;

            Self::handshake(&socket).await?;
            Self::get_model_definitions(&socket).await?
        };

        let socket = Arc::new(UdpSocket::bind((Ipv4Addr::UNSPECIFIED, DATA_PORT)).await?);

        let multicast_ip = Ipv4Addr::from_str(DEFAULT_MULTICAST_IP).unwrap();
        socket.join_multicast_v4(multicast_ip, Ipv4Addr::UNSPECIFIED)?;

        let task = tokio::spawn(Self::receiver_task(description, socket, record));

        Ok(MotionCapture { task })
    }

    async fn receiver_task(
        _descriptions: protocol::Description,
        socket: Arc<UdpSocket>,
        record: RecordHandle<DIMENSIONS>,
    ) -> Result<(), TaskError> {
        let mut buf = Vec::with_capacity(DEFAULT_BUFFER_CAPACITY);

        loop {
            socket
                .recv_buf(&mut buf)
                .await
                .map_err(|_| TaskError::SocketClosed)?;

            let frame = protocol::parse_frame_data(&buf).map_err(|_| TaskError::MalformedPacket)?;

            if let Some(rigid_body) = frame.rigid_bodies.first() {
                let p = &rigid_body.position;
                let q = &rigid_body.orientation;
                let data = [p.0, p.1, p.2, q.0, q.1, q.2, q.3];

                record.append(data).await;
            }

            buf.clear();
        }
    }

    async fn handshake(socket: &UdpSocket) -> io::Result<()> {
        tracing::info!("Sending handshake message...");

        let msg = protocol::handshake_msg();
        socket.send(&msg.packet()).await?;

        let mut buf = BytesMut::new();
        socket.recv(&mut buf).await?;

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

        let mut buf = BytesMut::new();
        socket.recv(&mut buf).await?;

        match protocol::parse_message(&mut Bytes::from(buf)) {
            Ok(msg) if msg.id == protocol::NAT_MODELDEF => {
                let mut payload = Bytes::from(msg.payload);

                match protocol::parse_description(&mut payload) {
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
