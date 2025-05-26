use std::{
    fmt,
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use async_trait::async_trait;
use defs::Command;
use eyre::Result;
use protocol::{Instruction, Link, NetFtApi2};
use tokio::{net::UdpSocket, sync::Mutex, task::JoinHandle};

use crate::{
    config::LoadCellConfig,
    data::sink::{DataSinkBuilder, StreamWriter},
    defs::Load,
    hardware::HardwareAgent,
};

pub mod defs;
mod protocol;

const PORT: u16 = 49152;

pub struct LoadCell {
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
    stream: Option<StreamWriter>,
}

impl LoadCell {
    pub const NAME: &str = "load_cell";
    pub const CHANNELS: [&str; Load::WIDTH] = Load::CHANNELS;

    const STANDARD_FORCE_UNIT: &str = "N";
    const STANDARD_TORQUE_UNIT: &str = "Nm";

    pub async fn try_new(ip: IpAddr) -> Result<Self> {
        let config = Link::fetch_config(ip).await?;

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
        socket.connect((ip, PORT)).await?;

        let link = Link::new(socket);

        let inner = Arc::new(Inner {
            config,
            link,
            shared: Mutex::new(Shared::default()),
        });

        let task = tokio::spawn(Self::load_cell_task(inner.clone()));

        Ok(LoadCell { inner, task })
    }

    pub async fn try_new_from_config(config: &LoadCellConfig) -> Result<Self> {
        Self::try_new(config.ip).await
    }

    pub async fn command(&self, command: Command) -> Result<()> {
        match command {
            Command::SetBias => {
                self.inner.instruction(Instruction::SetBias).await?;
            }
        }

        Ok(())
    }

    /* == Background tasks == */

    async fn load_cell_task(inner: Arc<Inner>) -> Result<()> {
        let mut buf = Vec::new();

        loop {
            let load = inner.link.receive_load(&mut buf, &inner.config).await?;
            let lock = inner.shared.lock().await;

            if let Some(stream) = &lock.stream {
                stream.add(&load.array()).await;
            }

            buf.clear();
        }
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
        let _ = self.inner.instruction(Instruction::StartStreaming).await;
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
