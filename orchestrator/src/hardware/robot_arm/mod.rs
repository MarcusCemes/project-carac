use std::{
    mem,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use bincode::{Decode, Encode};
use bytes::{Buf, BufMut};
use eyre::{bail, eyre, Report, Result};
use serde::{Deserialize, Serialize};
use tokio::{
    net::UdpSocket,
    select,
    sync::{oneshot, watch, Mutex},
    task::JoinHandle,
};

use crate::{
    config::RobotArmConfig,
    defs::*,
    misc::network_config,
    recording::{Sink, StreamWriter},
};

use super::Hardware;

const NAME: &str = "robot_arm";
const CHANNELS: [&str; 7] = ["x", "y", "z", "qx", "qy", "qz", "qw"];

const BUFFER_SIZE: usize = 1024;
const HANDSHAKE_REQUEST_ID: u8 = 0;
const MAGIC_HEADER: u8 = 0x95;

pub struct RobotArm {
    inner: Arc<Inner>,
    task: JoinHandle<Result<()>>,
}

struct Inner {
    error: watch::Sender<Option<u16>>,
    moving: watch::Sender<bool>,
    pose: watch::Sender<Option<Pose>>,
    shared: Mutex<Shared>,
    socket: UdpSocket,
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
    pub async fn connect_from_config(config: &RobotArmConfig) -> Result<Self> {
        Self::connect(config.ip, config.port).await
    }

    pub async fn connect(ip: IpAddr, port: u16) -> Result<RobotArm> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port)).await?;
        socket.connect((ip, port)).await?;

        Self::handshake(&socket).await?;

        let (error, _) = watch::channel(None);
        let (moving, _) = watch::channel(false);
        let (status, _) = watch::channel(None);

        let inner = Arc::new(Inner::new(
            error.clone(),
            moving.clone(),
            status.clone(),
            socket,
        ));

        let task = tokio::spawn(Self::robot_arm_task(inner.clone()));

        Ok(RobotArm { inner, task })
    }

    pub async fn register(&self, sink: &Sink) -> Result<()> {
        let channels = CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.add_stream(NAME.to_owned(), channels).await?;

        self.inner.shared.lock().await.stream = Some(stream);
        Ok(())
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

    pub fn controller(self) -> RobotController {
        RobotController(self)
    }

    /* == Commands == */

    pub async fn execute_command(&mut self, command: RobotCommand) -> Result<()> {
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
        let mut error = self.inner.error.subscribe();
        let mut moving = self.inner.moving.subscribe();

        let error_task = error.wait_for(Option::is_some);

        let moving_task = async {
            moving.wait_for(|m| *m).await?;
            moving.wait_for(|m| !*m).await?;
            Result::<(), watch::error::RecvError>::Ok(())
        };

        // Race the error and negative moving edge, whichever comes first
        select! {
            Ok(code) = error_task => Err(code.unwrap()),
            _ = moving_task => Ok(()),
        }
    }

    /* == Background tasks == */

    async fn robot_arm_task(inner: Arc<Inner>) -> Result<()> {
        let mut buf = Vec::with_capacity(BUFFER_SIZE);

        loop {
            inner.socket.recv_buf(&mut buf).await?;

            match Response::decode(&buf[..])? {
                Response::Status(report) => {
                    inner.moving.send_replace(true);
                    inner.pose.send_replace(Some(Pose::from(report.position)));
                }

                Response::Ack(id) => {
                    tracing::trace!("<- ACK {id}");
                    let mut lock = inner.shared.lock().await;

                    lock.ack
                        .take_if(|(ack_id, _)| *ack_id == id)
                        .and_then(|(_, ack)| ack.send(()).ok());
                }

                Response::Error(code) => {
                    tracing::warn!("Robot error (code {code})");
                    inner.error.send(Some(code)).ok();
                }

                Response::MotionComplete => {
                    tracing::debug!("Motion complete");
                    inner.moving.send(false).ok();
                }

                Response::PowerOff => {
                    tracing::info!("Power off");
                }
            }

            buf.clear();
        }
    }
}

impl Hardware for RobotArm {
    async fn errored(&mut self) -> Option<Report> {
        self.inner
            .error
            .borrow()
            .map(|code| eyre!("Robot error (code {code})"))
    }

    async fn on_record(&mut self) {
        let lock = self.inner.shared.lock().await;

        let maybe_pose = self.inner.pose.borrow();
        let maybe_stream = lock.stream.as_ref();
        let combined_option = maybe_stream.zip(maybe_pose.as_ref());

        if let Some((
            stream,
            Pose {
                position: p,
                orientation: o,
            },
        )) = combined_option
        {
            stream
                .write_now(&[p[0], p[1], p[2], o.1[0], o.1[1], o.1[2], o.0])
                .await;
        }
    }

    async fn reset(&mut self) {
        self.inner.error.send_replace(None);
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
        moving: watch::Sender<bool>,
        pose: watch::Sender<Option<Pose>>,
        socket: UdpSocket,
    ) -> Self {
        Inner {
            error,
            moving,
            pose,
            shared: Mutex::new(Shared::default()),
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

pub enum RobotCommand {
    Halt { return_home: bool },
    Hello,
    Move(Motion),
    ReturnHome(MotionType),
    SetReportInterval(f32),
    SetSpeedProfile(SpeedProfile),
    SetToolOffset(Point),
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub enum Motion {
    Direct(Point),
    Joint(Joint),
    Linear(Point),
}

#[derive(Copy, Clone, Debug)]
pub enum MotionType {
    Direct,
    Joint,
    Linear,
}

#[derive(Copy, Clone, Debug, Decode, Deserialize, Encode, Serialize)]
pub struct SpeedProfile {
    pub translation_limit: f32,
    pub rotation_limit: f32,
    pub acceleration_scale: u8,
    pub velocity_scale: u8,
    pub deceleration_scale: u8,
}

const SPEED_PROFILE_SIZE: usize = mem::size_of::<SpeedProfile>();

impl From<&Motion> for MotionType {
    fn from(value: &Motion) -> Self {
        match value {
            Motion::Direct(_) => MotionType::Direct,
            Motion::Joint(_) => MotionType::Joint,
            Motion::Linear(_) => MotionType::Linear,
        }
    }
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

        let response = match buf.get_u8() {
            0x01 => {
                let status = get_value(buf).unwrap();
                Response::Status(status)
            }

            0x02 => Response::MotionComplete,
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
