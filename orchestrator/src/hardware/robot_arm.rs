use std::time::Duration;

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

const ACK_TIMEOUT: Duration = Duration::from_secs(3);

pub struct RobotArm {
    ack: mpsc::Receiver<()>,
    moving: watch::Receiver<bool>,
    socket: OwnedWriteHalf,

    #[allow(dead_code)]
    task_handle: JoinSet<()>,
}

#[derive(Copy, Clone, Debug)]
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rx: f32,
    pub ry: f32,
    pub rz: f32,
}

pub struct MoveConfig {
    pub translation: f32,
    pub rotation: f32,
    pub acceleration_scale: u8,
    pub speed_scale: u8,
    pub deceleration_scale: u8,
}

#[derive(Copy, Clone, Debug)]
struct Joint([f32; 6]);

pub enum MoveType {
    Linear,
    Direct,
}

enum Response {
    Ack,
    Error(u16),
    MotionComplete,
    PowerOff,
    Report,
}

#[derive(Debug)]
enum ResponseError {
    InvalidMagicHeader,
    InvalidResponse,
}

const MAGIC_HEADER: u8 = 0x95;
const PORT: u16 = 20000;

impl RobotArm {
    pub async fn connect(addr: &str) -> io::Result<RobotArm> {
        let (socket_rx, socket) = TcpStream::connect((addr, PORT)).await?.into_split();

        let (task_handle, ack, moving) = Self::spawn_tasks(socket_rx);

        Ok(RobotArm {
            ack,
            moving,
            socket,
            task_handle,
        })
    }

    pub async fn move_position(&mut self, position: Position, move_type: MoveType) {
        let command = match move_type {
            MoveType::Linear => 0x02,
            MoveType::Direct => 0x03,
        };

        let payload = Vec::<u8>::from(position);

        self.send(command, &payload).await;
    }

    pub async fn return_home(&mut self) {
        self.send(0x08, &[]).await;
    }

    pub async fn wait_motion(&mut self) {
        loop {
            if !*self.moving.borrow() {
                break;
            }

            self.moving.changed().await.unwrap();
        }
    }

    pub async fn stop(&mut self) {
        self.send(0x01, &[0]).await;
    }

    pub async fn return_and_stop(&mut self) {
        self.send(0x01, &[1]).await;
    }

    async fn send(&mut self, command: u8, payload: &[u8]) {
        let mut buf = Vec::new();

        buf.push(MAGIC_HEADER);
        buf.push(command);
        buf.extend_from_slice(payload);

        self.socket
            .write_all(&buf)
            .await
            .expect("[ROBOT_ARM] Socket write failure");

        timeout(ACK_TIMEOUT, self.ack.recv())
            .await
            .expect("[ROBOT_ARM] ACK timeout");
    }

    /* == Background tasks == */

    fn spawn_tasks(
        socket: OwnedReadHalf,
    ) -> (JoinSet<()>, mpsc::Receiver<()>, watch::Receiver<bool>) {
        let (ack_tx, ack_rx) = mpsc::channel(1);
        let (moving_tx, moving_rx) = watch::channel(false);

        let mut set = JoinSet::new();

        set.spawn(Self::receiver(ack_tx, moving_tx, socket));

        (set, ack_rx, moving_rx)
    }

    async fn receiver(
        ack: mpsc::Sender<()>,
        moving: watch::Sender<bool>,
        mut socket: OwnedReadHalf,
    ) {
        loop {
            if socket.read_u8().await.unwrap() != MAGIC_HEADER {
                tracing::error!("Missing magic header");
                continue;
            }

            let response = Response::decode(&mut socket).await.unwrap();

            match response {
                Response::Report => {
                    moving.send(true).unwrap();
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
                Self::discard_payload(socket, 72).await;
                Ok(Response::Report)
            }
            0x02 => Ok(Response::MotionComplete),
            0x03 => Ok(Response::PowerOff),
            0xFE => Ok(Response::Ack),

            0xFF => {
                let error = socket.read_u16_le().await.unwrap();
                Ok(Response::Error(error))
            }

            _ => {
                tracing::error!("Invalid response {:02x}, draining TCP stream", code);
                Self::drain(socket).await;
                Err(ResponseError::InvalidResponse)
            }
        }
    }

    async fn discard_payload(socket: &mut OwnedReadHalf, len: usize) {
        io::copy(&mut socket.take(len as u64), &mut io::sink())
            .await
            .unwrap();
    }

    async fn drain(socket: &mut OwnedReadHalf) {
        socket.try_read_buf(&mut Vec::new()).unwrap();
    }
}

pub struct NotEnoughBytes;

impl TryFrom<&[u8]> for Position {
    type Error = NotEnoughBytes;

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        let mut it = value
            .chunks_exact(4)
            .map(|x| f32::from_le_bytes(x.try_into().unwrap()));

        let mut next = || it.next().ok_or(NotEnoughBytes);

        Ok(Position {
            x: next()?,
            y: next()?,
            z: next()?,
            rx: next()?,
            ry: next()?,
            rz: next()?,
        })
    }
}

impl From<Position> for Vec<u8> {
    fn from(value: Position) -> Self {
        [value.x, value.y, value.z, value.rx, value.ry, value.rz]
            .iter()
            .flat_map(|x| x.to_le_bytes())
            .collect()
    }
}
