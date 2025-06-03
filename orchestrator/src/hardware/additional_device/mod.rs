use std::{fmt, sync::Arc, time::Duration};

use async_trait::async_trait;
use eyre::Result;
use tokio::{
    sync::Mutex,
    task::JoinSet,
    time::{Instant, MissedTickBehavior, interval},
};

use crate::{
    config::DeviceConfig,
    data::sink::{DataSinkBuilder, StreamWriter},
    hardware::{HardwareAgent, additional_device::protocol::Link},
};

mod protocol;

pub enum Command {
    Set(Vec<f32>),
}

pub struct Device {
    inner: Arc<Inner>,
    tasks: JoinSet<Result<()>>,
}

struct Inner {
    config: DeviceConfig,
    link: Link,
    shared: Mutex<Shared>,
}

struct Shared {
    state: Box<[f32]>,
    stream: Option<StreamWriter>,
}

impl Device {
    pub async fn connect_from_config(config: &DeviceConfig) -> Result<Self> {
        Self::connect(config.clone()).await
    }

    pub async fn connect(config: DeviceConfig) -> Result<Self> {
        let link = Link::try_new(config.ip, config.port).await?;
        let inner = Arc::new(Inner::new(config, link));

        let mut tasks = JoinSet::new();

        tasks.spawn(Self::send_task(inner.clone()));
        tasks.spawn(Self::receive_task(inner.clone()));

        Ok(Device { inner, tasks })
    }

    pub fn config(&self) -> &DeviceConfig {
        &self.inner.config
    }

    pub async fn command(&self, mut command: Command) -> Result<()> {
        if let Some(bounds) = &self.inner.config.extra.channel_bounds {
            Self::apply_bounds(&mut command, bounds)?;
        }

        match command {
            Command::Set(state) => {
                {
                    let mut lock = self.inner.shared.lock().await;
                    lock.state.copy_from_slice(&state);
                }

                self.inner.link.send_state(&state, &mut Vec::new()).await?;
            }
        }

        Ok(())
    }

    fn apply_bounds(command: &mut Command, bounds: &[(f32, f32)]) -> Result<()> {
        match command {
            Command::Set(values) => {
                assert_eq!(values.len(), bounds.len(),);

                for (value, (min, max)) in values.iter_mut().zip(bounds) {
                    *value = value.clamp(*min, *max);
                }
            }
        }

        Ok(())
    }

    /* == Task == */

    async fn send_task(inner: Arc<Inner>) -> Result<()> {
        let maybe_frequency = inner.config.extra.transmit_rate;
        let maybe_period = maybe_frequency.map(|p| Duration::from_secs_f32(1. / p as f32));

        if let Some(period) = maybe_period {
            let mut buf = Vec::with_capacity(inner.config.channels.len());
            let mut timer = interval(period);

            timer.set_missed_tick_behavior(MissedTickBehavior::Skip);

            loop {
                timer.tick().await;

                let lock = inner.shared.lock().await;
                inner.link.send_state(&lock.state, &mut buf).await?;
            }
        }

        Ok(())
    }

    async fn receive_task(inner: Arc<Inner>) -> Result<()> {
        let mut buf = Vec::with_capacity(Link::BUFFER_SIZE);
        let mut data = Vec::with_capacity(inner.config.channels.len());

        loop {
            inner.link.receive_state(&mut data, &mut buf).await?;
            let now = Instant::now();

            let lock = inner.shared.lock().await;

            if let Some(stream) = &lock.stream {
                stream.add(now, &data).await;
            }

            buf.clear();
            data.clear();
        }
    }
}

#[async_trait]
impl HardwareAgent for Device {
    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        let DeviceConfig { name, channels, .. } = &self.inner.config;
        let stream = sink.register_stream(name.clone(), channels.clone()).await;

        let mut lock = self.inner.shared.lock().await;
        lock.stream = Some(stream);
    }
}

impl fmt::Display for Device {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Device ({})", self.inner.config.name)
    }
}

impl Inner {
    fn new(config: DeviceConfig, link: Link) -> Self {
        Self {
            shared: Mutex::new(Shared {
                state: vec![0.; config.channels.len()].into_boxed_slice(),
                stream: None,
            }),
            config,
            link,
        }
    }
}
