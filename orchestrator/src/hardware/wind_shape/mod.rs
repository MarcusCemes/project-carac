use std::{
    cmp, fmt,
    net::IpAddr,
    str,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU8, Ordering},
    },
    time::Duration,
};

use async_trait::async_trait;
use eyre::Result;
use tokio::{
    select,
    sync::{Mutex, watch},
    task::JoinSet,
    time::{Instant, MissedTickBehavior, interval},
};

use crate::{
    config::WindShapeConfig,
    data::{
        orchestrator::{Event, EventServer},
        sink::{DataSinkBuilder, StreamWriter},
    },
    hardware::HardwareAgent,
};

use self::{defs::*, protocol::*};

pub mod defs;
pub mod protocol;

pub struct WindShape {
    inner: Arc<Inner>,

    request_control: watch::Sender<bool>,
    status: watch::Sender<Status>,

    tasks: JoinSet<Result<()>>,
}

pub struct Inner {
    client_id: u8,
    link: Link,
    shared: Mutex<Shared>,
    state: State,
    virtual_speed: watch::Sender<u8>,
}

#[derive(Default)]
pub struct State {
    fan_speed: AtomicU8,
    powered: AtomicBool,
}

#[derive(Default)]
pub struct Shared {
    stream: Option<StreamWriter>,
}

impl WindShape {
    pub const NAME: &str = "wind";
    pub const CHANNELS: [&str; 1] = ["speed"];

    const LOCAL_PORT: u16 = 60333;
    const REMOTE_PORT: u16 = 60334;

    const MODULE_COUNT: usize = 56;
    const MODULE_FANS: usize = 18;

    const STATUS_INTERVAL: Duration = Duration::from_millis(40);
    const STREAM_INTERVAL: Duration = Duration::from_millis(250);

    pub async fn connect_from_config(config: &WindShapeConfig) -> Result<Self> {
        Self::connect(config.ip).await
    }

    pub async fn connect(ip: IpAddr) -> Result<WindShape> {
        let link = Link::try_new(ip).await?;

        let client_id = link.handshake().await?;

        let inner = Arc::new(Inner::new(client_id, link));

        let (status, request_control, tasks) = Self::spawn_tasks(inner.clone());

        Ok(WindShape {
            inner,

            request_control,

            status,
            tasks,
        })
    }

    /* == Public API == */

    pub async fn command(&self, command: Command, events: &EventServer) -> Result<()> {
        match command {
            Command::SetFanSpeed(speed) => {
                if speed > 0. {
                    events.publish(Event::Buzzer);
                }

                self.set_fan_speed(speed).await
            }
            Command::SetPowered(powered) => self.set_powered(powered).await,
            Command::WaitSettled => self.wait_settled().await,
        }
    }

    pub async fn request_control(&self) {
        self.set_in_control(true).await
    }

    pub async fn release_control(&self) {
        self.set_in_control(false).await
    }

    pub async fn set_powered(&self, powered: bool) -> Result<()> {
        self.inner.state.powered.store(powered, Ordering::Relaxed);
        self.send_module_state().await
    }

    pub async fn set_fan_speed(&self, fan_speed: f32) -> Result<()> {
        let fan_speed = (100. * fan_speed).round().clamp(0., 100.) as u8;

        self.inner
            .state
            .fan_speed
            .store(fan_speed, Ordering::Relaxed);

        self.send_module_state().await
    }

    async fn set_in_control(&self, value: bool) {
        self.request_control.send(value).unwrap();

        self.status
            .subscribe()
            .wait_for(|s| s.in_control == value)
            .await
            .unwrap();
    }

    async fn wait_settled(&self) -> Result<()> {
        let mut channel = self.inner.virtual_speed.subscribe();

        loop {
            channel.changed().await?;

            let set_speed = self.inner.state.fan_speed.load(Ordering::Relaxed);
            let virtual_speed = channel.borrow_and_update();

            if set_speed == *virtual_speed {
                return Ok(());
            }
        }
    }

    async fn send_module_state(&self) -> Result<()> {
        let fan_speed = self.inner.state.fan_speed.load(Ordering::Relaxed);
        let powered = self.inner.state.powered.load(Ordering::Relaxed);

        let request = Request::new(
            self.inner.client_id,
            Instruction::Module { powered, fan_speed },
        );

        self.inner.link.send_request(request).await
    }

    /* == Background tasks == */

    fn spawn_tasks(
        inner: Arc<Inner>,
    ) -> (
        watch::Sender<Status>,
        watch::Sender<bool>,
        JoinSet<Result<()>>,
    ) {
        let (status, _) = watch::channel(Status::default());
        let (state_tx, state_rx) = watch::channel(false);

        let mut set = JoinSet::new();

        set.spawn(Self::control_task(inner.clone(), state_rx));
        set.spawn(Self::receiver_task(inner.clone(), status.clone()));
        set.spawn(Self::stream_task(inner));

        (status, state_tx, set)
    }

    async fn control_task(inner: Arc<Inner>, mut control: watch::Receiver<bool>) -> Result<()> {
        let mut timer = interval(Self::STATUS_INTERVAL);

        timer.set_missed_tick_behavior(MissedTickBehavior::Skip);

        loop {
            // Wait for either the control state to change or the timer to tick
            select! {
                _ = control.changed() => {}
                _ = timer.tick() => {}
            }

            // Send a status message to the device
            let request_control = *control.borrow();
            let request = Request::new(inner.client_id, Instruction::Status { request_control });

            inner.link.send_request(request).await?;
        }
    }

    async fn receiver_task(inner: Arc<Inner>, status: watch::Sender<Status>) -> Result<()> {
        let mut buf = Vec::with_capacity(Link::RX_BUFFER_SIZE);

        loop {
            let response = inner.link.receive_response(&mut buf).await?;

            if response.client_id != inner.client_id {
                continue;
            }

            if let ResponsePayload::Status(Status { in_control }) = response.payload {
                status.send_modify(|s| s.in_control = in_control);
            }
        }
    }

    async fn stream_task(inner: Arc<Inner>) -> Result<()> {
        let mut fan = VirtualFan::new();
        let mut timer = interval(Self::STREAM_INTERVAL);

        timer.set_missed_tick_behavior(MissedTickBehavior::Skip);

        loop {
            timer.tick().await;
            let now = Instant::now();

            let set_speed = inner.state.fan_speed.load(Ordering::Relaxed);
            let speed = fan.update(set_speed, timer.period());

            let _ = inner.virtual_speed.send_replace(speed);

            if let Ok(shared) = inner.shared.try_lock() {
                if let Some(stream) = shared.stream.as_ref() {
                    let virtual_speed = speed as f32 / 100.;
                    stream.add(now, &[virtual_speed]).await;
                }
            }
        }
    }
}

impl fmt::Display for WindShape {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", Self::NAME)
    }
}

#[async_trait]
impl HardwareAgent for WindShape {
    async fn register(&self, sink: &mut DataSinkBuilder) {
        let name = Self::NAME.to_owned();
        let channels = Self::CHANNELS.map(str::to_owned).to_vec();

        let stream = sink.register_stream(name, channels).await;

        self.inner.shared.lock().await.stream = Some(stream);
    }

    async fn start(&self) {
        self.request_control().await;
    }

    async fn stop(&self) {
        self.release_control().await;
    }
}

impl Drop for WindShape {
    fn drop(&mut self) {
        self.tasks.abort_all();
    }
}

impl Inner {
    pub fn new(client_id: u8, link: Link) -> Self {
        Inner {
            client_id,
            link,
            shared: Mutex::default(),
            state: State::default(),
            virtual_speed: watch::channel(0).0,
        }
    }
}

/* == Miscellaneous == */

#[derive(Clone, Debug, Default)]
struct VirtualFan(u8);

impl VirtualFan {
    const FAN_CHANGE_RATE: u8 = 6;

    pub fn new() -> Self {
        VirtualFan::default()
    }

    pub fn update(&mut self, target: u8, delta: Duration) -> u8 {
        let delta = (delta.as_secs_f32() * Self::FAN_CHANGE_RATE as f32).round() as u8;
        let target = target.min(100);

        self.0 = match target.cmp(&self.0) {
            cmp::Ordering::Greater => self.0.saturating_add(delta).min(target),
            cmp::Ordering::Less => self.0.saturating_sub(delta).max(target),
            cmp::Ordering::Equal => self.0,
        };

        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_virtual_fan() {
        let mut fan = VirtualFan::new();

        let delta = Duration::from_secs(1);

        let sequence = [
            (0, 0),
            (50, 5),
            (100, 10),
            (50, 15),
            (0, 10),
            (0, 5),
            (0, 0),
            (0, 0),
        ];

        for (target, expected) in sequence {
            let result = fan.update(target, delta);
            assert_eq!(result, expected);
        }
    }
}
