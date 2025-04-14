use std::time::Duration;

use bincode::{config, Decode, Encode};
use tokio::{
    io::{self, AsyncReadExt, AsyncWriteExt},
    net::{
        tcp::{OwnedReadHalf, OwnedWriteHalf},
        TcpStream,
    },
    sync::{mpsc, watch},
    task::JoinSet,
    time::timeout,
};

use crate::{defs::*, recorder::StreamHandle};

const ACK_TIMEOUT: Duration = Duration::from_secs(3);
const DEFAULT_IP: &str = "192.168.100.254";
const DEFAULT_PORT: u16 = 20000;
const MAGIC_HEADER: u8 = 0x95;

pub struct RobotArm {
    ack: mpsc::Receiver<()>,
    moving: watch::Receiver<bool>,
    socket: OwnedWriteHalf,

    #[allow(dead_code)]
    task_handle: JoinSet<()>,
}

#[derive(Copy, Clone, Debug, Encode, Decode)]
pub struct MotionConfig {
    pub translation: f32,
    pub rotation: f32,
    pub acceleration_scale: u8,
    pub speed_scale: u8,
    pub deceleration_scale: u8,
}

pub enum Motion {
    Joint(Point),
    Linear(Point),
    Pose(Joint),
}

enum Response {
    Ack,
    Error(u16),
    MotionComplete,
    PowerOff,
    Report(Report),
}

#[derive(Decode)]
struct Report {
    position: Point,
    pose: Joint,
    error: Joint,
}

#[derive(Debug)]
struct ResponseError;

impl RobotArm {
    pub async fn connect(
        ip: Option<&str>,
        port: Option<u16>,
        record: Option<StreamHandle>,
    ) -> io::Result<RobotArm> {
        let ip = ip.unwrap_or(DEFAULT_IP);
        let port = port.unwrap_or(DEFAULT_PORT);

        let (socket_rx, socket) = TcpStream::connect((ip, port)).await?.into_split();

        let (task_handle, ack, moving) = Self::spawn_tasks(socket_rx, record);

        Ok(RobotArm {
            ack,
            moving,
            socket,
            task_handle,
        })
    }

    /* == Commands == */

    pub async fn move_to(&mut self, motion: &Motion) {
        match motion {
            Motion::Linear(point) => {
                let payload = bincode::encode_to_vec(point, config::standard()).unwrap();
                self.send_command(0x02, &payload).await;
            }

            Motion::Joint(joint) => {
                let payload = bincode::encode_to_vec(joint, config::standard()).unwrap();
                self.send_command(0x03, &payload).await;
            }

            Motion::Pose(joint) => {
                let payload = bincode::encode_to_vec(joint, config::standard()).unwrap();
                self.send_command(0x04, &payload).await;
            }
        }
    }

    pub async fn set_config(&mut self, config: &MotionConfig) {
        let payload = bincode::encode_to_vec(config, config::standard()).unwrap();
        self.send_command(0x05, &payload).await;
    }

    pub async fn set_offset(&mut self, offset: &Point) {
        let payload = bincode::encode_to_vec(offset, config::standard()).unwrap();
        self.send_command(0x06, &payload).await;
    }

    pub async fn set_report_time(&mut self, time: f32) {
        let payload = time.to_le_bytes();
        self.send_command(0x07, &payload).await;
    }

    pub async fn return_home(&mut self) {
        self.send_command(0x08, &[2]).await;
    }

    pub async fn wait_motion(&mut self) {
        loop {
            if *self.moving.borrow() {
                break;
            }

            self.moving.changed().await.unwrap();
        }

        loop {
            if !*self.moving.borrow() {
                break;
            }

            self.moving.changed().await.unwrap();
        }
    }

    pub async fn halt(&mut self) {
        self.send_command(0x01, &[0]).await;
    }

    pub async fn return_and_stop(&mut self) {
        self.send_command(0x01, &[1]).await;
    }

    async fn send_command(&mut self, command: u8, payload: &[u8]) {
        let mut buf = Vec::new();

        buf.push(MAGIC_HEADER);
        buf.push(command);
        buf.extend_from_slice(payload);

        self.socket
            .write_all(&buf)
            .await
            .expect("Socket write failure");

        timeout(ACK_TIMEOUT, self.ack.recv())
            .await
            .expect("ACK timeout");
    }

    /* == Background tasks == */

    fn spawn_tasks(
        socket: OwnedReadHalf,
        record: Option<StreamHandle>,
    ) -> (JoinSet<()>, mpsc::Receiver<()>, watch::Receiver<bool>) {
        let (ack_tx, ack_rx) = mpsc::channel(1);
        let (moving_tx, moving_rx) = watch::channel(false);

        let mut set = JoinSet::new();

        set.spawn(Self::receiver_task(ack_tx, moving_tx, socket, record));

        (set, ack_rx, moving_rx)
    }

    async fn receiver_task(
        ack: mpsc::Sender<()>,
        moving: watch::Sender<bool>,
        mut socket: OwnedReadHalf,
        record: Option<StreamHandle>,
    ) {
        loop {
            if socket.read_u8().await.unwrap() != MAGIC_HEADER {
                tracing::error!("Missing magic header");
                continue;
            }

            let response = Response::decode(&mut socket).await.unwrap();

            match response {
                Response::Report(report) => {
                    moving.send(true).unwrap();

                    if let Some(ref record) = record {
                        let pose = Pose::from(report.position);
                        record.add(&pose.to_array()).await;
                    }
                }

                Response::Ack => {
                    ack.try_send(()).expect("Ack channel closed");
                }

                Response::Error(code) => {
                    tracing::error!("Error response: {}", code);
                }

                Response::MotionComplete => {
                    tracing::info!("Motion complete");
                    moving.send(false).unwrap();
                }

                Response::PowerOff => {
                    tracing::info!("Power off");
                }
            }
        }
    }
}

impl Response {
    async fn decode(socket: &mut OwnedReadHalf) -> Result<Response, ResponseError> {
        let code = socket.read_u8().await.unwrap();

        match code {
            0x01 => {
                let mut buf = [0u8; 72];

                socket
                    .read_exact(&mut buf)
                    .await
                    .expect("Socket read failure");

                let (report, _) = bincode::decode_from_slice(&buf, config::standard()).unwrap();
                Ok(Response::Report(report))
            }
            0x02 => Ok(Response::MotionComplete),
            0x03 => Ok(Response::PowerOff),
            0xFE => Ok(Response::Ack),

            0xFF => {
                let error = socket.read_u16_le().await.unwrap();
                Ok(Response::Error(error))
            }

            _ => {
                tracing::error!("Invalid response {:02x}. Draining TCP stream", code);
                Self::drain(socket).await;
                Err(ResponseError)
            }
        }
    }

    async fn drain(socket: &mut OwnedReadHalf) {
        socket.try_read_buf(&mut Vec::new()).unwrap();
    }
}
