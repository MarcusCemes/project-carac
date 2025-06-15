use std::{
    array,
    f32::consts::PI,
    fmt,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
    time::Duration,
};

use async_trait::async_trait;
use eyre::{Report, Result, bail};
use nalgebra::{Isometry3, Point3, Vector3};
use thiserror::Error;
use tokio::{
    net::UdpSocket,
    select,
    sync::{Mutex, oneshot, watch},
    task::JoinHandle,
    time::{Instant, timeout},
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
mod protocol;

/* === Definitions === */

pub struct RobotArm {
    inner: Arc<Inner>,

    receiver: JoinHandle<Result<()>>,
    _watchdog: JoinHandle<Result<()>>,
}

struct Inner {
    bounds: Option<Bounds>,
    error: watch::Sender<Option<RobotError>>,
    shared: Mutex<Shared>,
    socket: UdpSocket,
    state: watch::Sender<Option<State>>,
}

#[derive(Default)]
struct Shared {
    ack: Option<(u8, oneshot::Sender<()>)>,
    command_counter: u8,
    origin_transform: Isometry3<f32>,
    stream: Option<StreamWriter>,
}

#[derive(Copy, Clone, Debug, Error)]
pub enum RobotError {
    #[error("Collision detected")]
    Collision,

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

    pub async fn try_new(ip: IpAddr, port: u16, bounds: Option<Bounds>) -> Result<RobotArm> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port)).await?;
        socket.connect((ip, port)).await?;

        Self::handshake(&socket).await?;

        let (error, _) = watch::channel(None);
        let (state, _) = watch::channel(None);

        let inner = Arc::new(Inner::new(bounds, error, socket, state));
        let receiver = tokio::spawn(Self::receiver_task(inner.clone()));
        let _watchdog = tokio::spawn(Self::collision_task(inner.clone()));

        Ok(RobotArm {
            inner,
            receiver,
            _watchdog,
        })
    }

    pub async fn try_new_from_config(config: &RobotArmConfig) -> Result<Self> {
        Self::try_new(config.ip, config.port, config.bounds).await
    }

    async fn handshake(socket: &UdpSocket) -> Result<()> {
        tracing::debug!("Handshaking with robot...");

        let mut buf = Vec::with_capacity(Self::BUFFER_SIZE);

        Request::new(Self::HANDSHAKE_REQUEST_ID, Command::Hello).encode(&mut buf);

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

    pub async fn command(&self, command: Command) -> Result<()> {
        match command {
            Command::SetOrigin(point) => {
                let isometry = Isometry3::from(point);

                let mut lock = self.inner.shared.lock().await;
                lock.origin_transform = isometry.inverse();

                Ok(())
            }

            Command::WaitSettled => {
                self.try_wait_settled().await?;
                Ok(())
            }

            Command::WaitProgress(progress) => {
                self.wait_for_progress(progress).await?;
                Ok(())
            }

            remote_command => self.instruction(remote_command).await,
        }
    }

    /* == Instructions == */

    pub async fn instruction(&self, remote_command: Command) -> Result<()> {
        let mut buf = Vec::with_capacity(Self::BUFFER_SIZE);

        let (request_id, ack) = self.next_request_id().await;

        Request::new(request_id, remote_command).encode(&mut buf);

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
            let now = Instant::now();

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

                Response::State(mut state) => {
                    // Swap the X/Y axis to match the drone's coordinate system
                    Self::tool_to_drone_frame(&mut state.position);

                    let lock = inner.shared.lock().await;

                    // Subtract the stored origin transform for relative positioning
                    let isometry = Isometry3::from(state.position);
                    let transformed = lock.origin_transform * isometry;
                    state.position = Point::from(transformed);

                    if let Some(stream) = &lock.stream {
                        stream.add(now, &state.position.array()).await;
                    }

                    inner.state.send_replace(Some(state));
                }

                Response::Settled(settled) => {
                    inner.state.send_if_modified(|state| {
                        state.as_mut().is_some_and(|state| {
                            let notify = state.settled != settled;
                            state.settled = settled;
                            notify
                        })
                    });
                }
            }
        }
    }

    async fn collision_task(inner: Arc<Inner>) -> Result<()> {
        let Some(bounds) = inner.bounds else {
            tracing::warn!("Robot bounds not set, watchdog inactive");
            return Ok(());
        };

        let mut channel = inner.state.subscribe();

        loop {
            // Subscribe to the state channel and wait for a position
            // that collides with the bounds
            channel
                .wait_for(|state| {
                    state
                        .as_ref()
                        .is_some_and(|state| bounds.collision_x(&state.position))
                })
                .await?;

            tracing::error!("Robot collision detected, halting");
            let mut buf = Vec::with_capacity(Self::BUFFER_SIZE);

            // Immediately halt the robot, engaging the brakes
            let request_id = inner.shared.lock().await.next_id();

            Request::new(request_id, Command::Halt(false)).encode(&mut buf);

            inner.socket.send(&buf).await?;

            // Set the error state and wait for it to be cleared
            inner.error.send_replace(Some(RobotError::Collision));
            inner.error.subscribe().wait_for(Option::is_none).await?;
        }
    }

    /* == Miscellaneous == */

    pub fn get_progress(&self) -> Option<f32> {
        self.inner.state.borrow().as_ref().map(|s| s.progress)
    }

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

            Ok(())
        };

        // Race the error and negative moving edge, whichever comes first
        select! {
            Ok(error) = error_task => Err(error),
            _ = moving_task() => Ok(()),
        }
    }

    async fn wait_for_progress(&self, value: f32) -> Result<(), RobotError> {
        let mut error = self.inner.error.subscribe();
        let mut state = self.inner.state.subscribe();

        let error_task = async {
            error.wait_for(Option::is_some).await.map(|maybe_error| {
                // SAFETY: wait_for guarantees that the value is Some
                unsafe { maybe_error.unwrap_unchecked() }
            })
        };

        let mut progress_task = async || -> Result<()> {
            state
                .wait_for(|s| s.as_ref().is_some_and(|s| s.progress >= value))
                .await?;

            Ok(())
        };

        // Race the error and progress value, whichever comes first
        select! {
            Ok(error) = error_task => Err(error),
            _ = progress_task() => Ok(()),
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

    /// Convert the robot's mm/deg -> m/rad units, while rotating 90 degrees
    /// counter-clockwise around the z-axis to match the drone's coordinate system.
    fn tool_to_drone_frame(point: &mut Point) {
        point.position *= 1e-3; // mm -> m
        point.orientation *= PI / 180.; // deg -> rad

        (point.position.x, point.position.y) = (point.position.y, -point.position.x);
        (point.orientation.x, point.orientation.y) = (point.orientation.y, -point.orientation.x);
    }
}

#[async_trait]
impl HardwareAgent for RobotArm {
    async fn error(&self) -> Result<(), Report> {
        match *self.inner.error.borrow() {
            Some(code) => bail!("Robot error (code {code})"),
            None => Ok(()),
        }
    }

    async fn start(&self) {
        for i in [
            Command::SetReporting(true),
            Command::SetProfile(Profile::default()),
        ] {
            let _ = self.instruction(i).await;
        }
    }

    async fn stop(&self) {
        let _ = self.instruction(Command::SetReporting(false)).await;
    }

    async fn register(&self, sink: &mut DataSinkBuilder) {
        let name = Self::NAME.to_owned();
        let channels = Self::CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.register_stream(name, channels).await;

        self.inner.shared.lock().await.stream = Some(stream);
    }

    async fn clear_error(&self) {
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
        bounds: Option<Bounds>,
        error: watch::Sender<Option<RobotError>>,
        socket: UdpSocket,
        state: watch::Sender<Option<State>>,
    ) -> Self {
        let shared = Mutex::new(Shared::default());

        Inner {
            bounds,
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

impl Bounds {
    pub fn collision_x(&self, origin: &Point) -> bool {
        let isometry = Isometry3::from_parts(origin.position.into(), origin.quaternion());

        self.iter_vertices()
            .iter()
            .any(|v| (isometry * v).x >= self.wall_distance_x)
    }

    fn iter_vertices(&self) -> [Point3<f32>; 8] {
        Self::sign_permutations().map(|v| Point3::from(v.component_mul(&self.size)))
    }

    fn sign_permutations() -> [Vector3<f32>; 8] {
        array::from_fn(|i| Vector3::new(Self::bit(i, 0), Self::bit(i, 1), Self::bit(i, 2)))
    }

    #[inline(always)]
    const fn bit(value: usize, bit: u8) -> f32 {
        if value & (1 << bit) == 0 { 1.0 } else { -1.0 }
    }
}

#[cfg(test)]
mod tests {

    use std::f32::consts::*;

    use super::*;

    #[test]
    fn test_sign_permutations() {
        let result = Bounds {
            size: Vector3::new(1., 2., 3.),
            wall_distance_x: 0.,
        }
        .iter_vertices()
        .map(|v| v.coords.data.0[0]);

        assert_eq!(
            result,
            [
                [1., 2., 3.],
                [-1., 2., 3.],
                [1., -2., 3.],
                [-1., -2., 3.],
                [1., 2., -3.],
                [-1., 2., -3.],
                [1., -2., -3.],
                [-1., -2., -3.]
            ]
        );
    }

    #[test]
    fn collisions() {
        let bounds = Bounds {
            wall_distance_x: 2.,
            size: Vector3::new(1., 2., 20.),
        };

        let cases = [
            (0., 0., false),
            (0.99, 0., false),
            (1.01, 0., true),
            (-0.13, FRAC_PI_4, false),
            (-0.12, FRAC_PI_4, true),
        ];

        for (x, rz, collision) in cases {
            assert_eq!(
                bounds.collision_x(&Point::new(x, 0., 0., 0., 0., rz)),
                collision,
                "Collision check failed for x: {}, rz: {}",
                x,
                rz
            );
        }
    }
}
