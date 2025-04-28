use std::{
    mem,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use bincode::{Decode, Encode};
use bytes::{Buf, BufMut};
use eyre::{bail, eyre, Result};
use tokio::{
    net::UdpSocket,
    select,
    sync::{oneshot, watch, Mutex},
    task::JoinHandle,
};

use crate::{defs::*, recording::StreamWriter};

const BUFFER_SIZE: usize = 1024;
const HANDSHAKE_REQUEST_ID: u8 = 0;
const MAGIC_HEADER: u8 = 0x95;

pub struct RobotArm {
    inner: Arc<Inner>,
    task: JoinHandle<Result<()>>,
}

struct Inner {
    error: watch::Receiver<Option<u16>>,
    moving: watch::Receiver<bool>,
    shared: Mutex<Shared>,
    socket: UdpSocket,
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
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port)).await?;
        socket.connect((ip, port)).await?;

        Self::handshake(&socket).await?;

        let (error_tx, error_rx) = watch::channel(None);
        let (moving_tx, moving_rx) = watch::channel(false);
        let (status_tx, status_rx) = watch::channel(None);

        let inner = Arc::new(Inner::new(error_rx, moving_rx, status_rx, socket));

        let task = tokio::spawn(Self::robot_arm_task(
            inner.clone(),
            error_tx,
            moving_tx,
            status_tx,
        ));

        Ok(RobotArm { inner, task })
    }

    async fn handshake(socket: &UdpSocket) -> Result<()> {
        let mut buf = Vec::with_capacity(BUFFER_SIZE);
        RobotCommand::Hello.encode(&mut buf, HANDSHAKE_REQUEST_ID);

        tracing::debug!("Handshaking with robot...");
        socket.send(&buf).await?;

        loop {
            buf.clear();

            socket.recv_buf_from(&mut buf).await?;

            match Response::decode(&buf[..])? {
                Response::Ack(0) => break,
                _ => (),
            }
        }

        tracing::info!("Handshake successful!");

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
        let (ack_tx, ack) = oneshot::channel();

        let request_id = {
            let mut lock = self.inner.shared.lock().await;
            let request_id = lock.request_id();
            lock.ack = Some((request_id, ack_tx));
            request_id
        };

        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        tracing::trace!("-> {request_id}");
        command.encode(&mut buf, request_id);
        self.inner.socket.send(&buf).await?;

        ack.await.ok();

        Ok(())
    }

    pub async fn try_wait_settled(&self) -> Result<(), u16> {
        let mut error = self.inner.error.clone();
        let mut moving = self.inner.moving.clone();

        let mut i = 0;
        let sequence = [true, false];

        // Wait for a negative moving edge or an error code
        loop {
            select! {
                _ = error.changed() => {
                    if let Some(code) = *error.borrow() {
                        return Err(code);
                    }
                }

                _ = moving.changed() => {
                    if *moving.borrow() == sequence[i] {
                        i += 1;
                    }

                    if i == sequence.len() {
                        return Ok(());
                    }
                }


            }
        }
    }

    /* == Background tasks == */

    async fn robot_arm_task(
        inner: Arc<Inner>,
        error: watch::Sender<Option<u16>>,
        moving: watch::Sender<bool>,
        status: watch::Sender<Option<RobotStatus>>,
    ) -> Result<()> {
        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        loop {
            inner.socket.recv_buf(&mut buf).await?;

            match Response::decode(&buf[..])? {
                Response::Status(report) => {
                    moving.send(true)?;
                    status.send(Some(report))?;
                }

                Response::Ack(id) => {
                    tracing::trace!("<- ACK {id}");

                    let mut lock = inner.shared.lock().await;

                    if let Some((_, ack)) = lock.ack.take_if(|(ack_id, _)| *ack_id == id) {
                        ack.send(()).ok();
                    }
                }

                Response::Error(code) => {
                    tracing::warn!("Robot error (code {code})");
                    error.send(Some(code)).ok();
                }

                Response::MotionComplete => {
                    tracing::info!("Motion complete");
                    moving.send(false)?;
                }

                Response::PowerOff => {
                    tracing::info!("Power off");
                }
            }

            buf.clear();
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
        error: watch::Receiver<Option<u16>>,
        moving: watch::Receiver<bool>,
        status: watch::Receiver<Option<RobotStatus>>,
        socket: UdpSocket,
    ) -> Self {
        Inner {
            error,
            moving,
            shared: Mutex::new(Shared::default()),
            status,
            socket,
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
    fn encode<B: BufMut>(&self, b: &mut B, request_id: u8) {
        b.put_u8(MAGIC_HEADER);
        b.put_u8(request_id);

        match self {
            RobotCommand::Hello => {
                b.put_u8(0x00);
            }

            RobotCommand::Halt { return_home } => {
                b.put_u8(0x01);
                b.put_u8(*return_home as u8);
            }

            RobotCommand::Move(motion) => {
                b.put_u8(0x02 + MotionType::from(motion).id());

                match motion {
                    Motion::Linear(point) | Motion::Direct(point) => put_value(b, point),
                    Motion::Joint(joint) => put_value(b, joint),
                }
            }

            RobotCommand::SetSpeedProfile(profile) => {
                b.put_u8(0x05);
                put_value(b, profile);
            }

            RobotCommand::SetToolOffset(offset) => {
                b.put_u8(0x06);
                put_value(b, offset);
            }

            RobotCommand::SetReportInterval(interval_s) => {
                b.put_u8(0x07);
                b.put_f32(*interval_s);
            }

            RobotCommand::ReturnHome(motion_type) => {
                b.put_u8(0x08);
                b.put_u8(*motion_type as u8);
            }
        };
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

    pub async fn wait_settled(&self) -> Result<()> {
        self.0
            .try_wait_settled()
            .await
            .map_err(|code| eyre!("Robot error (code {code})"))
    }
}

impl MotionType {
    pub fn id(&self) -> u8 {
        match self {
            MotionType::Linear => 0,
            MotionType::Direct => 1,
            MotionType::Joint => 2,
        }
    }
}

/* == Response == */

impl Response {
    fn decode<B: Buf>(mut buf: B) -> Result<Response> {
        let buf = &mut buf;

        if buf.get_u8() != MAGIC_HEADER {
            bail!("Missing magic header");
        }

        match buf.get_u8() {
            0x01 => {
                let status = get_value(buf).unwrap();
                Ok(Response::Status(status))
            }

            0x02 => Ok(Response::MotionComplete),
            0x03 => Ok(Response::PowerOff),

            0xFE => {
                let id = buf.get_u8();
                Ok(Response::Ack(id))
            }

            0xFF => {
                let error = buf.get_u16();
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
        .with_big_endian()
        .with_fixed_int_encoding()
}

fn get_value<B: Buf, T: Decode<()>>(buf: &mut B) -> Result<T> {
    let value = bincode::decode_from_std_read(&mut buf.reader(), binary_config())?;
    Ok(value)
}

fn put_value<B: BufMut, T: Encode>(buf: &mut B, value: &T) {
    bincode::encode_into_std_write(value, &mut buf.writer(), binary_config()).unwrap();
}
