use std::{io, mem, net::IpAddr, sync::Arc, time::Duration};

use bincode::{Decode, Encode};
use eyre::{bail, Result};
use tokio::{
    io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt, BufStream, WriteHalf},
    net::TcpStream,
    sync::{oneshot, watch, Mutex},
    task::JoinHandle,
};

use crate::{defs::*, recording::StreamWriter};

const ACK_TIMEOUT: Duration = Duration::from_secs(3);
const MAGIC_HEADER: u8 = 0x95;

pub struct RobotArm {
    inner: Arc<Inner>,
    socket: WriteHalf<BufStream<TcpStream>>,
    task: JoinHandle<Result<()>>,
}

struct Inner {
    moving: watch::Receiver<bool>,
    shared: Mutex<Shared>,
    status: watch::Receiver<Option<RobotStatus>>,
}

#[derive(Default)]
struct Shared {
    ack: Option<(u8, oneshot::Sender<()>)>,
    command_counter: u8,
    stream: Option<StreamWriter>,
}

enum Response {
    Ack(u8),
    Error(u16),
    MotionComplete,
    PowerOff,
    Status(RobotStatus),
}

#[derive(Decode)]
struct RobotStatus {
    position: Point,
    pose: Joint,
    error: Joint,
}

impl RobotArm {
    pub async fn connect(ip: IpAddr, port: u16) -> Result<RobotArm> {
        let socket = TcpStream::connect((ip, port)).await?;
        let mut socket = BufStream::new(socket);

        Self::handshake(&mut socket).await?;

        let (socket_rx, socket) = tokio::io::split(socket);

        let (moving_tx, moving_rx) = watch::channel(false);
        let (status_tx, status_rx) = watch::channel(None);

        let inner = Arc::new(Inner::new(moving_rx, status_rx));

        let task = tokio::spawn(Self::robot_arm_task(
            inner.clone(),
            moving_tx,
            status_tx,
            socket_rx,
        ));

        Ok(RobotArm {
            inner,
            socket,
            task,
        })
    }

    async fn handshake<S: AsyncRead + AsyncWrite + Unpin>(s: &mut S) -> Result<()> {
        RobotCommand::Hello.write_to(s, 0).await?;

        let expected_sequence = [MAGIC_HEADER, 0xFE, 0];

        // Drain the stream until we find the magic header and ACK code
        'outer: loop {
            for value in expected_sequence {
                if value != s.read_u8().await? {
                    continue 'outer;
                }
            }

            break;
        }

        Ok(())
    }

    pub async fn set_stream_writer(&self, handle: StreamWriter) {
        self.inner.shared.lock().await.stream = Some(handle)
    }

    pub fn controller(self) -> RobotController {
        RobotController(self)
    }

    /* == Commands == */

    pub async fn execute_command(&mut self, command: RobotCommand<'_>) -> Result<()> {
        let (ack_tx, ack_rx) = oneshot::channel();

        let request_id = {
            let mut lock = self.inner.shared.lock().await;
            let request_id = lock.request_id();
            lock.ack = Some((request_id, ack_tx));
            request_id
        };

        command.write_to(&mut self.socket, request_id).await?;
        ack_rx.await.ok();
        Ok(())
    }

    pub async fn wait_settled(&self) {
        let mut channel = self.inner.moving.clone();

        for value in [true, false] {
            loop {
                if *channel.borrow() == value {
                    break;
                }

                channel.changed().await.unwrap();
            }
        }
    }

    /* == Background tasks == */

    async fn robot_arm_task<R: AsyncRead + Unpin>(
        inner: Arc<Inner>,
        moving: watch::Sender<bool>,
        status: watch::Sender<Option<RobotStatus>>,
        mut r: R,
    ) -> Result<()> {
        loop {
            match Response::read_from(&mut r).await? {
                Response::Status(report) => {
                    moving.send(true)?;
                    status.send(Some(report))?;
                }

                Response::Ack(id) => {
                    let mut lock = inner.shared.lock().await;

                    if let Some((_, ack)) = lock.ack.take_if(|(ack_id, _)| *ack_id == id) {
                        ack.send(()).ok();
                    }
                }

                Response::Error(code) => {
                    tracing::error!("Error response: {}", code);
                }

                Response::MotionComplete => {
                    tracing::info!("Motion complete");
                    moving.send(false)?;
                }

                Response::PowerOff => {
                    tracing::info!("Power off");
                }
            }
        }
    }
}

impl Drop for RobotArm {
    fn drop(&mut self) {
        self.task.abort();
    }
}

impl Inner {
    pub fn new(
        moving: watch::Receiver<bool>,
        status: watch::Receiver<Option<RobotStatus>>,
    ) -> Self {
        Inner {
            moving,
            shared: Mutex::new(Shared::default()),
            status,
        }
    }
}

impl Shared {
    fn request_id(&mut self) -> u8 {
        self.command_counter = self.command_counter.wrapping_add(1);
        self.command_counter
    }
}

/* == Command == */

pub enum RobotCommand<'a> {
    Halt { return_home: bool },
    Hello,
    Move(Motion<'a>),
    ReturnHome(MotionType),
    SetReportInterval(f32),
    SetSpeedProfile(&'a SpeedProfile),
    SetToolOffset(&'a Point),
}

#[derive(Copy, Clone, Debug)]
pub enum Motion<'a> {
    Direct(&'a Point),
    Joint(&'a Joint),
    Linear(&'a Point),
}

#[derive(Copy, Clone, Debug)]
pub enum MotionType {
    Direct,
    Joint,
    Linear,
}

#[derive(Encode)]
pub struct SpeedProfile {
    pub translation_limit: f32,
    pub rotation_limit: f32,
    pub acceleration_scale: u8,
    pub velocity_scale: u8,
    pub deceleration_scale: u8,
}

const SPEED_PROFILE_SIZE: usize = mem::size_of::<SpeedProfile>();

impl From<&Motion<'_>> for MotionType {
    fn from(value: &Motion) -> Self {
        match value {
            Motion::Direct(_) => MotionType::Direct,
            Motion::Joint(_) => MotionType::Joint,
            Motion::Linear(_) => MotionType::Linear,
        }
    }
}

/* == Command == */

impl RobotCommand<'_> {
    async fn write_to<W: AsyncWrite + Unpin>(&self, w: &mut W, request_id: u8) -> io::Result<()> {
        w.write_u8(MAGIC_HEADER).await?;
        w.write_u8(request_id).await?;

        match self {
            RobotCommand::Hello => {
                w.write_u8(0x00).await?;
            }

            RobotCommand::Halt { return_home } => {
                w.write_u8(0x01).await?;
                w.write_u8(*return_home as u8).await?;
            }

            RobotCommand::Move(motion) => {
                w.write_u8(0x02 + MotionType::from(motion) as u8).await?;

                match motion {
                    Motion::Linear(point) | Motion::Direct(point) => {
                        write_sized_value(w, point).await?;
                    }

                    Motion::Joint(joint) => write_sized_value(w, joint).await?,
                }
            }

            RobotCommand::SetSpeedProfile(profile) => {
                w.write_u8(0x05).await?;
                write_sized_value(w, profile).await?;
            }

            RobotCommand::SetToolOffset(offset) => {
                w.write_u8(0x06).await?;
                write_sized_value(w, offset).await?;
            }

            RobotCommand::SetReportInterval(interval_s) => {
                w.write_u8(0x07).await?;
                w.write_f32_le(*interval_s).await?;
            }

            RobotCommand::ReturnHome(motion_type) => {
                w.write_u8(0x08).await?;
                w.write_u8(*motion_type as u8).await?;
            }
        };

        Ok(())
    }
}

/* == Controller == */

pub struct RobotController(RobotArm);

impl RobotController {
    pub async fn move_to(&mut self, motion: Motion<'_>) -> Result<()> {
        self.0.execute_command(RobotCommand::Move(motion)).await
    }

    pub async fn set_offset(&mut self, offset: &Point) -> Result<()> {
        self.0
            .execute_command(RobotCommand::SetToolOffset(offset))
            .await
    }

    pub async fn set_profile(&mut self, profile: &SpeedProfile) -> Result<()> {
        self.0
            .execute_command(RobotCommand::SetSpeedProfile(profile))
            .await
    }

    pub async fn go_home(&mut self, motion_type: MotionType) -> Result<()> {
        self.0
            .execute_command(RobotCommand::ReturnHome(motion_type))
            .await
    }

    pub async fn halt(&mut self, return_home: bool) -> Result<()> {
        self.0
            .execute_command(RobotCommand::Halt { return_home })
            .await
    }

    pub async fn wait_settled(&self) {
        self.0.wait_settled().await
    }
}

/* == Response == */

impl Response {
    async fn read_from<R: AsyncRead + Unpin>(r: &mut R) -> Result<Response> {
        if r.read_u8().await? != MAGIC_HEADER {
            bail!("Missing magic header");
        }

        match r.read_u8().await? {
            0x01 => {
                let status = read_sized_value(r).await?;
                Ok(Response::Status(status))
            }

            0x02 => Ok(Response::MotionComplete),
            0x03 => Ok(Response::PowerOff),

            0xFE => {
                let id = r.read_u8().await?;
                Ok(Response::Ack(id))
            }

            0xFF => {
                let error = r.read_u16_le().await?;
                Ok(Response::Error(error))
            }

            code => {
                tracing::error!("Unsupported code: {code:02x}");
                bail!("Unsupported code: {code:02x}");
            }
        }
    }
}

/* == Protocol == */

fn binary_config() -> impl bincode::config::Config {
    bincode::config::standard()
        .with_little_endian()
        .with_fixed_int_encoding()
}

async fn read_sized_value<T, R>(r: &mut R) -> Result<T>
where
    T: Decode<()>,
    R: AsyncRead + Unpin,
{
    let mut buf = vec![0u8; mem::size_of::<T>()];
    r.read_exact(&mut buf).await?;

    let (value, _) = bincode::decode_from_slice(&buf, binary_config())?;
    Ok(value)
}

pub async fn write_sized_value<T, W>(w: &mut W, value: &T) -> io::Result<()>
where
    T: Encode,
    W: AsyncWrite + Unpin,
{
    let buf = bincode::encode_to_vec(value, binary_config()).unwrap();
    w.write_all(&buf).await?;

    Ok(())
}
