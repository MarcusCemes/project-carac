use std::{
    fmt,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use async_trait::async_trait;
use defs::Command;
use eyre::Result;
use protocol::{Instruction, Link, LoadCounts, NetFtApi2};
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle, time::Instant};

use crate::{
    config::LoadCellConfig,
    data::{
        processing::LoadTransform,
        sink::{DataSinkBuilder, StreamWriter},
    },
    defs::Load,
    hardware::HardwareAgent,
};

pub mod defs;
mod protocol;

pub struct LoadCell {
    buffered_streaming: bool,
    inner: Arc<Inner>,
    task: JoinHandle<Result<()>>,
}

struct Inner {
    config: NetFtApi2,
    link: Link,
    shared: Mutex<Shared>,
}

#[derive(Default)]
struct Shared {
    transform: LoadTransform,
    stream: Option<StreamWriter>,
}

impl LoadCell {
    pub const NAME: &str = "load";
    pub const CHANNELS: [&str; Load::WIDTH] = Load::CHANNELS;

    const STANDARD_FORCE_UNIT: &str = "N";
    const STANDARD_TORQUE_UNIT: &str = "Nm";

    pub async fn try_new(ip: IpAddr, buffered_streaming: bool) -> Result<Self> {
        let config = Link::fetch_config(ip).await?;

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
        socket.connect((ip, Link::PORT)).await?;

        let link = Link::new(LoadCounts::from(&config), socket);

        let inner = Arc::new(Inner {
            config,
            link,
            shared: Mutex::new(Shared::default()),
        });

        let task = tokio::spawn(Self::load_cell_task(inner.clone()));

        Ok(LoadCell {
            buffered_streaming,
            inner,
            task,
        })
    }

    pub async fn try_new_from_config(config: &LoadCellConfig) -> Result<Self> {
        let load_cell = Self::try_new(config.ip, config.buffered_streaming).await?;

        if config.configure_device {
            load_cell.update_settings().await?;
        }

        if let Some(point) = config.transform {
            tracing::info!("Applying load transform from config");
            load_cell.inner.shared.lock().await.transform = LoadTransform::new(&point);
        }

        Ok(load_cell)
    }

    pub async fn update_settings(&self) -> Result<()> {
        for (page, variables) in [
            ("setting", [("setuserfilter", "0")].as_slice()),
            (
                "config",
                [
                    ("cfgid", "15"),
                    ("cfgfu", "2"),
                    ("cfgtu", "3"),
                    ("cfgtdu", "5"),
                    ("cfgtau", "1"),
                    ("cfgtfx0", "0.0"),
                    ("cfgtfx1", "0.0"),
                    ("cfgtfx2", "0.0"),
                    ("cfgtfx3", "0.0"),
                    ("cfgtfx4", "0.0"),
                    ("cfgtfx5", "0.0"),
                ]
                .as_slice(),
            ),
            (
                "comm",
                [
                    ("comrdte", "1"),
                    ("comrdtrate", "7000"),
                    ("comrdtbsiz", "40"),
                ]
                .as_slice(),
            ),
        ] {
            self.inner
                .link
                .set_variables(page, variables.iter())
                .await?;
        }

        tracing::info!("Remote load cell settings updated");

        Ok(())
    }

    pub async fn command(&self, command: Command) -> Result<()> {
        match command {
            Command::SetBias => {
                self.inner.instruction(Instruction::SetBias).await?;
            }

            Command::SetTransform(point) => {
                self.inner.shared.lock().await.transform = LoadTransform::new(&point);
            }
        }

        Ok(())
    }

    /* == Background tasks == */

    async fn load_cell_task(inner: Arc<Inner>) -> Result<()> {
        let mut buf = Vec::with_capacity(Link::BUFFER_SIZE);
        let mut channel_data = Vec::new();

        loop {
            // Receives a packet and creates an iterator of decoded loads
            let iter_loads = inner.link.receive_loads(&mut buf).await?;

            let now = Instant::now();
            let lock = inner.shared.lock().await;

            if let Some(stream) = &lock.stream {
                // 1. Correct the orientation of the load vector (tool side)
                // 2. Apply the force/moment transform to the desired centre of mass
                let mut iter_loads = iter_loads.map(|mut load| {
                    LoadCell::adjust_load_orientation(&mut load);
                    lock.transform.apply(&load)
                });

                match iter_loads.len() {
                    0 => panic!("No loads"),

                    1 => stream.add(now, &iter_loads.next().unwrap().array()).await,

                    _ => {
                        channel_data.clear();

                        for load in iter_loads {
                            channel_data.extend_from_slice(&load.array());
                        }

                        stream
                            .add_many(now, inner.config.output_rate as f32, &channel_data)
                            .await;
                    }
                }
            }
        }
    }

    /* == Other == */

    /// Transforms the load vector from the device's coordinate system (X axis pointing
    /// towards the back of the device) into the drone's coordinate system, by rotating
    /// clockwise by 90 degrees around the Z axis.
    fn adjust_load_orientation(load: &mut Load) {
        (load.force.x, load.force.y) = (-load.force.y, load.force.x);
        (load.moment.x, load.moment.y) = (-load.moment.y, load.moment.x);
    }
}

#[async_trait]
impl HardwareAgent for LoadCell {
    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        let name = Self::NAME.to_owned();
        let channels = Self::CHANNELS.map(str::to_owned).to_vec();
        let stream = sink.register_stream(name, channels).await;

        self.inner.shared.lock().await.stream = Some(stream);
    }

    async fn bias(&mut self) {
        let _ = self.command(Command::SetBias).await;
    }

    async fn start(&mut self) {
        let instruction = match self.buffered_streaming {
            true => {
                tracing::info!("Starting buffered streaming");
                Instruction::StartBuffered
            }
            false => {
                tracing::info!("Starting unbuffered streaming");
                Instruction::StartStreaming
            }
        };

        let _ = self.inner.instruction(instruction).await;
    }

    async fn stop(&mut self) {
        let _ = self.inner.instruction(Instruction::StopStreaming).await;
    }
}

impl fmt::Display for LoadCell {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", Self::NAME)
    }
}

impl Drop for LoadCell {
    fn drop(&mut self) {
        self.task.abort();

        let inner = self.inner.clone();

        tokio::spawn(async move {
            let _ = inner.instruction(Instruction::StopStreaming).await;
        });
    }
}

impl Inner {
    async fn instruction(&self, instruction: Instruction) -> Result<()> {
        self.link.send_instruction(instruction).await?;
        Ok(())
    }
}
