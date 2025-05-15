use std::{
    fmt,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
    time::Duration,
};

use async_trait::async_trait;
use bincode::{Decode, Encode};
use bytes::{Buf, BufMut};
use eyre::{Report, Result, bail, eyre};
use serde::{Deserialize, Serialize};
use strum::{AsRefStr, EnumDiscriminants};
use thiserror::Error;
use tokio::{
    net::UdpSocket,
    select,
    sync::{Mutex, oneshot, watch},
    task::JoinHandle,
    time::timeout,
};

use crate::{
    config::RobotArmConfig,
    data::sink::{DataSinkBuilder, StreamWriter},
    defs::*,
    hardware::HardwareAgent,
    misc::network_config,
};

const NAME: &str = "robot_arm";
const CHANNELS: [&str; 7] = ["x", "y", "z", "qx", "qy", "qz", "qw"];

const BUFFER_SIZE: usize = 1024;
const HANDSHAKE_REQUEST_ID: u8 = 0;
const MAGIC_HEADER: u8 = 0x95;

const MAX_SETTLE_WAIT_MS: u64 = 100;

pub struct RobotArm {
    inner: Arc<Inner>,
    task: JoinHandle<Result<()>>,
}

struct Inner {
    error: watch::Sender<Option<u16>>,
    shared: Mutex<Shared>,
    socket: UdpSocket,
    state: watch::Sender<Option<State>>,
}

#[derive(Default)]
struct Shared {
    ack: Option<(u8, oneshot::Sender<()>)>,
    command_counter: u8,
    stream: Option<StreamWriter>,
}

#[derive(Debug)]
enum Response {
    Ack(u8),
    Error(u16),
    Motion(MotionState),
    PowerOff,
    Status(State),
}

#[derive(Debug, Decode)]
struct State {
    motion: MotionState,
    position: Point,
    pose: Joint,
    error: Joint,
}

#[derive(Debug, Decode)]
struct MotionState {
    settled: bool,
    idle: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum RobotArmInstruction {
    Move(Motion),
    SetOffset(Point),
    SetProfile(SpeedProfile),
    WaitSettled,
}

#[derive(Debug, Error)]
pub enum RobotArmError {
    #[error("Robot error (code {0})")]
    RemoteError(u16),
}

impl RobotArm {
    pub async fn connect_from_config(config: &RobotArmConfig) -> Result<Self> {
        Self::connect(config.ip, config.port).await
    }

    pub async fn connect(ip: IpAddr, port: u16) -> Result<RobotArm> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port)).await?;
        socket.connect((ip, port)).await?;

        Self::handshake(&socket).await?;

        let (error, _) = watch::channel(None);
        let (state, _) = watch::channel(None);

        let inner = Arc::new(Inner::new(error, socket, state));
        let task = tokio::spawn(Self::robot_arm_task(inner.clone()));

        Ok(RobotArm { inner, task })
    }

    pub async fn execute(&mut self, instruction: RobotArmInstruction) -> Result<()> {
        let command = match instruction {
            RobotArmInstruction::Move(motion) => RobotCommand::Move(motion),
            RobotArmInstruction::SetOffset(offset) => RobotCommand::SetToolOffset(offset),
            RobotArmInstruction::SetProfile(profile) => RobotCommand::SetSpeedProfile(profile),
            RobotArmInstruction::WaitSettled => return Ok(self.try_wait_settled().await?),
        };

        self.execute_command(command).await
    }

    async fn handshake(socket: &UdpSocket) -> Result<()> {
        let mut buf = Vec::with_capacity(BUFFER_SIZE);
        RobotCommand::Hello.encode(&mut buf, HANDSHAKE_REQUEST_ID);

        tracing::debug!("Handshaking with robot...");
        socket.send(&buf).await?;

        loop {
            buf.clear();

            socket.recv_buf_from(&mut buf).await?;

            if let Response::Ack(0) = Response::decode(&buf[..])? {
                break;
            }
        }

        tracing::info!("Handshake successful!");
        Ok(())
    }

    pub async fn set_stream_writer(&self, handle: StreamWriter) {
        self.inner.shared.lock().await.stream = Some(handle)
    }

    pub fn controller(&self) -> RobotController {
        RobotController(self)
    }

    /* == Commands == */

    pub async fn execute_command(&self, command: RobotCommand) -> Result<()> {
        let (ack_tx, ack) = oneshot::channel();

        let request_id = {
            let mut lock = self.inner.shared.lock().await;
            let request_id = lock.request_id();
            lock.ack = Some((request_id, ack_tx));
            request_id
        };

        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        tracing::trace!("-> {} ({request_id})", command.as_ref());
        command.encode(&mut buf, request_id);
        self.inner.socket.send(&buf).await?;

        ack.await.ok();

        Ok(())
    }

    pub async fn try_wait_settled(&self) -> Result<(), RobotArmError> {
        let mut error = self.inner.error.subscribe();
        let mut state = self.inner.state.subscribe();

        let error_task = error.wait_for(Option::is_some);

        let mut moving_task = async || -> Result<()> {
            timeout(
                Duration::from_millis(MAX_SETTLE_WAIT_MS),
                Self::wait_for_settled_value(&mut state, false),
            )
            .await??;

            Self::wait_for_settled_value(&mut state, true).await?;
            Ok(())
        };

        // Race the error and negative moving edge, whichever comes first
        select! {
            Ok(code) = error_task => Err(RobotArmError::RemoteError(code.unwrap())),
            _ = moving_task() => Ok(()),
        }
    }

    // async fn wait_positive_settle_edge(receiver: &mut watch::Receiver<Op>)

    async fn wait_for_settled_value(
        receiver: &mut watch::Receiver<Option<State>>,
        value: bool,
    ) -> Result<(), watch::error::RecvError> {
        receiver
            .wait_for(|s| s.as_ref().is_some_and(|s| s.motion.settled == value))
            .await?;

        Ok(())
    }

    /* == Background tasks == */

    async fn robot_arm_task(inner: Arc<Inner>) -> Result<()> {
        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        loop {
            inner.socket.recv_buf(&mut buf).await?;

            match Response::decode(&buf[..])? {
                Response::Status(state) => {
                    if let Some(stream) = &inner.shared.lock().await.stream {
                        let pose = Pose::from(&state.position);
                        stream.add(&pose.to_array()).await;
                    }

                    inner.state.send_replace(Some(state));
                }

                Response::Ack(id) => {
                    tracing::trace!("<- Ack ({id})");
                    let mut lock = inner.shared.lock().await;

                    lock.ack
                        .take_if(|(ack_id, _)| *ack_id == id)
                        .and_then(|(_, ack)| ack.send(()).ok());
                }

                Response::Error(code) => {
                    tracing::warn!("Robot error (code {code})");
                    inner.error.send(Some(code)).ok();
                }

                Response::Motion(motion) => {
                    tracing::debug!("Settled: {}", motion.settled);

                    inner.state.send_modify(|maybe_state| {
                        if let Some(state) = maybe_state {
                            state.motion = motion;
                        }
                    });
                }

                Response::PowerOff => {
                    tracing::info!("Power off");
                }
            }

            buf.clear();
        }
    }
}

#[async_trait]
impl HardwareAgent for RobotArm {
    async fn error(&mut self) -> Result<(), Report> {
        match *self.inner.error.borrow() {
            Some(code) => bail!("Robot error (code {code})"),
            None => Ok(()),
        }
    }

    async fn start(&mut self) {
        tracing::debug!("Requesting robot reporting");

        self.execute_command(RobotCommand::SetReporting(true))
            .await
            .ok();
    }

    async fn stop(&mut self) {
        tracing::debug!("Stopping robot reporting");
        self.execute_command(RobotCommand::SetReporting(false))
            .await
            .ok();
    }

    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        let name = NAME.to_owned();
        let channels = CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.register_stream(name, channels).await;

        self.inner.shared.lock().await.stream = Some(stream);
    }

    async fn reset_error(&mut self) {
        self.inner.error.send_replace(None);
    }
}

impl fmt::Display for RobotArm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{NAME}")
    }
}

impl Drop for RobotArm {
    fn drop(&mut self) {
        self.task.abort();
    }
}

impl Inner {
    pub fn new(
        error: watch::Sender<Option<u16>>,
        socket: UdpSocket,
        state: watch::Sender<Option<State>>,
    ) -> Self {
        Inner {
            error,
            shared: Mutex::new(Shared::default()),
            socket,
            state,
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

#[derive(AsRefStr)]
pub enum RobotCommand {
    Halt { return_home: bool },
    Hello,
    Move(Motion),
    ReturnHome(MotionDiscriminants),
    SetReportInterval(f32),
    SetSpeedProfile(SpeedProfile),
    SetToolOffset(Point),
    SetReporting(bool),
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize, EnumDiscriminants)]
pub enum Motion {
    Direct(Point),
    Joint(Joint),
    Linear(Point),
}

#[derive(Copy, Clone, Debug, Decode, Deserialize, Encode, Serialize)]
pub struct SpeedProfile {
    pub translation_limit: f32,
    pub rotation_limit: f32,
    pub acceleration_scale: u8,
    pub velocity_scale: u8,
    pub deceleration_scale: u8,
}

/* == Command == */

impl RobotCommand {
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
                b.put_u8(0x02 + MotionDiscriminants::from(motion).id());

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

            RobotCommand::SetReporting(enabled) => {
                b.put_u8(0x09);
                b.put_u8(*enabled as u8);
            }
        };
    }
}

/* == Controller == */

pub struct RobotController<'a>(&'a RobotArm);

impl RobotController<'_> {
    pub async fn move_to(&mut self, motion: Motion) -> Result<()> {
        self.0.execute_command(RobotCommand::Move(motion)).await
    }

    pub async fn set_offset(&mut self, offset: Point) -> Result<()> {
        self.0
            .execute_command(RobotCommand::SetToolOffset(offset))
            .await
    }

    pub async fn set_profile(&mut self, profile: SpeedProfile) -> Result<()> {
        self.0
            .execute_command(RobotCommand::SetSpeedProfile(profile))
            .await
    }

    pub async fn go_home(&mut self, motion_type: MotionDiscriminants) -> Result<()> {
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
        if self.0.inner.state.borrow().is_none() {
            tracing::warn!("Robot is not reporting state! This may not complete.")
        }

        self.0
            .try_wait_settled()
            .await
            .map_err(|code| eyre!("Robot error (code {code})"))
    }
}

impl MotionDiscriminants {
    pub fn id(&self) -> u8 {
        match self {
            MotionDiscriminants::Linear => 0,
            MotionDiscriminants::Direct => 1,
            MotionDiscriminants::Joint => 2,
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

        let response = match buf.get_u8() {
            0x01 => {
                let status = get_value(buf).unwrap();
                Response::Status(status)
            }

            0x02 => Response::Motion(MotionState {
                settled: buf.get_u8() != 0,
                idle: buf.get_u8() != 0,
            }),

            0x03 => Response::PowerOff,

            0xFE => {
                let id = buf.get_u8();
                Response::Ack(id)
            }

            0xFF => {
                let error = buf.get_u16();
                Response::Error(error)
            }

            code => {
                tracing::error!("Unsupported code: {code:02x}");
                bail!("Unsupported code: {code:02x}");
            }
        };

        Ok(response)
    }
}

/* == Protocol == */

fn get_value<B: Buf, T: Decode<()>>(buf: &mut B) -> Result<T> {
    let value = bincode::decode_from_std_read(&mut buf.reader(), network_config())?;
    Ok(value)
}

fn put_value<B: BufMut, T: Encode>(buf: &mut B, value: &T) {
    bincode::encode_into_std_write(value, &mut buf.writer(), network_config()).unwrap();
}
