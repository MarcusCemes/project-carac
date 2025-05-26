use std::{
    fmt,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
    time::Duration,
};

use async_trait::async_trait;
use eyre::{Report, Result, bail};
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
    misc::buf::{Decode, Encode},
};

use self::{defs::*, protocol::*};

pub mod defs;
pub mod protocol;

/* === Definitions === */

pub struct RobotArm {
    inner: Arc<Inner>,

    receiver: JoinHandle<Result<()>>,
    watchdog: JoinHandle<Result<()>>,
}

struct Inner {
    error: watch::Sender<Option<RobotError>>,
    shared: Mutex<Shared>,
    socket: UdpSocket,
    state: watch::Sender<Option<State>>,
}

#[derive(Default)]
struct Shared {
    ack: Option<(u8, oneshot::Sender<()>)>,
    command_counter: u8,
    origin: Point,
    stream: Option<StreamWriter>,
}

#[derive(Copy, Clone, Debug, Error)]
pub enum RobotError {
    #[error("Robot unreachable")]
    Unreachable,

    #[error("Robot powered off")]
    PoweredOff,

    #[error("Robot error (code {0})")]
    Remote(u16),
}

/* === Implementations === */

impl RobotArm {
    pub const NAME: &str = "robot";
    pub const CHANNELS: [&str; Point::WIDTH] = Point::CHANNELS;

    pub const HOME_POINT: Point = Point::new(50., 50., 300., 0., 0., 0.);
    pub const WORK_POINT: Point = Point::new(1400., 50., 300., 0., 0., 0.);

    const BUFFER_SIZE: usize = 1024;
    const HANDSHAKE_REQUEST_ID: u8 = 0;
    const MAX_SETTLE: Duration = Duration::from_millis(100);

    pub async fn try_new(ip: IpAddr, port: u16) -> Result<RobotArm> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port)).await?;
        socket.connect((ip, port)).await?;

        Self::handshake(&socket).await?;

        let (error, _) = watch::channel(None);
        let (state, _) = watch::channel(None);

        let inner = Arc::new(Inner::new(error, socket, state));
        let receiver = tokio::spawn(Self::receiver_task(inner.clone()));
        let watchdog = tokio::spawn(Self::watchdog_task(inner.clone()));

        Ok(RobotArm {
            inner,
            receiver,
            watchdog,
        })
    }

    pub async fn try_new_from_config(config: &RobotArmConfig) -> Result<Self> {
        Self::try_new(config.ip, config.port).await
    }

    async fn handshake(socket: &UdpSocket) -> Result<()> {
        tracing::debug!("Handshaking with robot...");

        let mut buf = Vec::with_capacity(Self::BUFFER_SIZE);

        Request::new(Self::HANDSHAKE_REQUEST_ID, Instruction::Hello).encode(&mut buf);

        socket.send(&buf).await?;

        loop {
            buf.clear();

            socket.recv_buf(&mut buf).await?;

            if let Response::Ack(0) = Response::decode(&mut &buf[..])? {
                break;
            }
        }

        tracing::info!("Handshake successful");
        Ok(())
    }

    /* == Commands == */

    pub async fn command(&mut self, command: Command) -> Result<()> {
        match command {
            Command::Remote(instruction) => self.instruction(instruction).await,

            Command::SetOrigin(point) => {
                self.inner.shared.lock().await.origin = point;
                Ok(())
            }

            Command::WaitSettled => {
                self.try_wait_settled().await?;
                Ok(())
            }
        }
    }

    /* == Instructions == */

    pub async fn instruction(&self, instruction: Instruction) -> Result<()> {
        let mut buf = Vec::with_capacity(Self::BUFFER_SIZE);

        let (request_id, ack) = self.next_request_id().await;

        Request::new(request_id, instruction).encode(&mut buf);

        self.inner.socket.send(&buf).await?;

        ack.await.ok();

        Ok(())
    }

    /* == Background tasks == */

    async fn receiver_task(inner: Arc<Inner>) -> Result<()> {
        let mut buf = Vec::with_capacity(Self::BUFFER_SIZE);

        loop {
            buf.clear();

            inner.socket.recv_buf(&mut buf).await?;

            match Response::decode(&mut &buf[..])? {
                Response::Ack(id) => {
                    tracing::trace!("<- Ack ({id})");
                    let mut lock = inner.shared.lock().await;

                    lock.ack
                        .take_if(|(ack_id, _)| *ack_id == id)
                        .and_then(|(_, ack)| ack.send(()).ok());
                }

                Response::Error(code) => {
                    let _ = inner.error.send(Some(RobotError::Remote(code)));
                }

                Response::PowerOff => {
                    let _ = inner.error.send(Some(RobotError::PoweredOff));
                }

                Response::State(state) => {
                    if let Some(stream) = &inner.shared.lock().await.stream {
                        stream.add(&state.position.array()).await;
                    }

                    inner.state.send_replace(Some(state));
                }
            }
        }
    }

    async fn watchdog_task(_inner: Arc<Inner>) -> Result<()> {
        tracing::warn!("Watchdog not implemented!");
        Ok(())
    }

    /* == Miscellaneous == */

    pub async fn try_wait_settled(&self) -> Result<(), RobotError> {
        let mut error = self.inner.error.subscribe();
        let mut state = self.inner.state.subscribe();

        let error_task = async {
            error.wait_for(Option::is_some).await.map(|maybe_error| {
                // SAFETY: wait_for guarantees that the value is Some
                unsafe { maybe_error.unwrap_unchecked() }
            })
        };

        let mut moving_task = async || -> Result<()> {
            timeout(
                Self::MAX_SETTLE,
                Self::wait_for_settled_value(&mut state, false),
            )
            .await??;

            Self::wait_for_settled_value(&mut state, true).await?;

            tracing::debug!("Motion settled");
            Ok(())
        };

        // Race the error and negative moving edge, whichever comes first
        select! {
            Ok(error) = error_task => Err(error),
            _ = moving_task() => Ok(()),
        }
    }

    async fn wait_for_settled_value(
        receiver: &mut watch::Receiver<Option<State>>,
        value: bool,
    ) -> Result<(), watch::error::RecvError> {
        receiver
            .wait_for(|s| s.as_ref().is_some_and(|s| s.settled == value))
            .await?;

        Ok(())
    }

    async fn next_request_id(&self) -> (u8, oneshot::Receiver<()>) {
        let (ack_tx, ack_rx) = oneshot::channel();

        let mut lock = self.inner.shared.lock().await;
        let request_id = lock.next_id();

        lock.ack = Some((request_id, ack_tx));

        (request_id, ack_rx)
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
        for i in [
            Instruction::SetReporting(true),
            Instruction::SetProfile(Profile::default()),
        ] {
            let _ = self.instruction(i).await;
        }
    }

    async fn stop(&mut self) {
        let _ = self.instruction(Instruction::SetReporting(false)).await;
    }

    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        let name = Self::NAME.to_owned();
        let channels = Self::CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.register_stream(name, channels).await;

        self.inner.shared.lock().await.stream = Some(stream);
    }

    async fn clear_error(&mut self) {
        self.inner.error.send_replace(None);
    }
}

impl fmt::Display for RobotArm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", Self::NAME)
    }
}

impl Drop for RobotArm {
    fn drop(&mut self) {
        self.receiver.abort();
    }
}

impl Inner {
    pub fn new(
        error: watch::Sender<Option<RobotError>>,
        socket: UdpSocket,
        state: watch::Sender<Option<State>>,
    ) -> Self {
        let shared = Mutex::new(Shared::default());

        Inner {
            error,
            shared,
            socket,
            state,
        }
    }
}

impl Shared {
    fn next_id(&mut self) -> u8 {
        self.command_counter = self.command_counter.wrapping_add(1);
        self.command_counter
    }
}
