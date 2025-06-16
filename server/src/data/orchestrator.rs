use std::time::Duration;

use eyre::{ContextCompat, Result, eyre};
use serde::{Deserialize, Serialize};
use tokio::{sync::Mutex, time::sleep};

use crate::{
    audio::AudioPlayer,
    config::Config,
    data::{experiment::Run, session::Session, sink::DataSink},
    hardware::{
        HardwareAgent, HardwareContext, additional_device::Command as DeviceCommand,
        load_cell::defs::Command as LoadCommand, robot_arm::defs::Command as RobotCommand,
        wind_shape::defs::Command as WindCommand,
    },
    misc::{plot_juggler::PlotJugglerBroadcaster, type_name},
};

/* === Definitions === */

pub struct Orchestrator {
    context: HardwareContext,
    events: EventServer,
    session: Mutex<Session>,
    sink: DataSink,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum Instruction {
    Device(String, Vec<f32>),
    Load(LoadCommand),
    Robot(RobotCommand),
    Wind(WindCommand),
    BiasAll,
    Sleep(Duration),
    Reset,
}

/* === Implementations === */

impl Orchestrator {
    pub async fn try_new(config: Config, mut context: HardwareContext) -> Result<Self> {
        let path = config.sink.session_path.wrap_err("Session path not set")?;

        let (sink, streams) = DataSink::builder().with_context(&mut context).await.build();

        if let Some(config) = config.sink.plot_juggler {
            tracing::info!("Enabling PlotJuggler broadcasting");

            let try_broadcaster = PlotJugglerBroadcaster::builder()
                .with_config(&config)
                .build();

            if let Ok(broadcaster) = try_broadcaster {
                sink.set_broadcaster(broadcaster).await;
            }
        }

        let events = EventServer::new(!config.sink.disable_audio);

        tracing::info!("Creating session");
        let session = Mutex::new(Session::open(path, streams)?);

        Ok(Self {
            context,
            events,
            session,
            sink,
        })
    }

    pub async fn start(&self) {
        for device in self.context.iter() {
            tracing::debug!("Starting agent {device}");
            device.start().await;
        }
    }

    pub async fn stop(&self) {
        for device in self.context.iter() {
            tracing::debug!("Stopping agent {device}");
            device.stop().await;
        }
    }

    pub fn context(&self) -> &HardwareContext {
        &self.context
    }

    pub async fn execute(&self, instructions: Vec<Instruction>) -> Result<()> {
        for instruction in instructions {
            self.instruction(instruction).await?;
        }

        Ok(())
    }

    pub async fn record(&self, instructions: Vec<Instruction>) -> Result<Run> {
        self.sink.start_recording().await;

        self.execute(instructions).await.inspect_err(|_| {
            self.events.publish(Event::Error);
        })?;

        let run = self.sink.stop_recording().await;
        self.events.publish(Event::Complete);

        let mut session = self.session.lock().await;
        session.append_run(&run).await?;

        Ok(run)
    }

    async fn instruction(&self, instruction: Instruction) -> Result<()> {
        tracing::trace!("Executing {instruction:?}");

        match instruction {
            Instruction::Device(name, command) => {
                self.context
                    .additional_devices
                    .iter()
                    .find(|device| device.config().name == name)
                    .ok_or_else(|| eyre!("Device {name} not found"))?
                    .command(DeviceCommand::Set(command))
                    .await?;
            }

            Instruction::Load(command) => {
                require_agent(&self.context.load_cell)?
                    .command(command)
                    .await?;
            }

            Instruction::Robot(command) => {
                require_agent(&self.context.robot_arm)?
                    .command(command)
                    .await?;
            }

            Instruction::Wind(command) => {
                require_agent(&self.context.wind_shape)?
                    .command(command, &self.events)
                    .await?;
            }

            Instruction::BiasAll => {
                for device in self.context.iter() {
                    device.bias().await;
                }
            }

            Instruction::Sleep(duration) => {
                sleep(duration).await;
            }

            Instruction::Reset => {
                for device in self.context.iter() {
                    device.clear_error().await;
                }
            }
        }

        Ok(())
    }

    pub async fn new_experiment(&self, name: String) -> Result<()> {
        self.session.lock().await.new_experiment(name)
    }

    pub async fn save_experiment(&self) -> Result<()> {
        self.session.lock().await.save_experiment()
    }

    pub async fn start_recording(&self) {
        self.sink.start_recording().await;
    }

    pub async fn stop_recording(&self) -> Result<Run> {
        let run = self.sink.stop_recording().await;
        self.events.publish(Event::Complete);

        let mut session = self.session.lock().await;
        session.append_run(&run).await?;

        Ok(run)
    }

    pub fn get_progress(&self) -> Result<f32> {
        Ok(require_agent(&self.context.robot_arm)?
            .get_progress()
            .unwrap_or(0.))
    }
}

pub struct EventServer {
    player: Option<AudioPlayer>,
}

#[derive(Copy, Clone, Debug)]
pub enum Event {
    Buzzer,
    Complete,
    End,
    Error,
    Notification,
}

impl EventServer {
    pub fn new(enable_audio: bool) -> Self {
        let player = enable_audio
            .then_some(())
            .and_then(|_| AudioPlayer::try_new().ok());

        Self { player }
    }

    pub fn publish(&self, event: Event) {
        if let Some(player) = &self.player {
            let _ = player.queue(event);
        }
    }
}

/* == Misc == */

fn require_agent<T: HardwareAgent>(module: &Option<T>) -> Result<&T> {
    module
        .as_ref()
        .ok_or_else(|| eyre!("Hardware {} not available", type_name::<T>()))
}
