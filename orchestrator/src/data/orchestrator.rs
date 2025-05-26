use std::time::Duration;

use eyre::{ContextCompat, Result, eyre};
use serde::{Deserialize, Serialize};
use tokio::time::sleep;

use crate::{
    audio::{AudioFile, AudioPlayer},
    config::Config,
    data::{experiment::Run, session::Session, sink::DataSink},
    hardware::{
        HardwareAgent, HardwareContext, load_cell::defs::Command as LoadCommand,
        robot_arm::defs::Command as RobotCommand, wind_shape::defs::Command as WindCommand,
    },
    misc::{plot_juggler::PlotJugglerBroadcaster, type_name},
};

/* === Definitions === */

pub struct Orchestrator {
    audio: Option<AudioPlayer>,
    context: HardwareContext,
    session: Session,
    sink: DataSink,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum Instruction {
    LoadCell(LoadCommand),
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
            let try_broadcaster = PlotJugglerBroadcaster::builder()
                .with_config(&config)
                .build();

            if let Ok(broadcaster) = try_broadcaster {
                sink.set_broadcaster(broadcaster).await;
            }
        }

        let audio = (!config.sink.disable_audio)
            .then_some(())
            .and_then(|_| AudioPlayer::try_new().ok());

        let session = Session::open(path, streams).await?;

        Ok(Self {
            audio,
            context,
            session,
            sink,
        })
    }

    pub async fn start(&mut self) {
        for device in self.context.iter_mut() {
            device.start().await;
        }
    }

    pub async fn stop(&mut self) {
        for device in self.context.iter_mut() {
            device.stop().await;
        }
    }

    pub fn context(&mut self) -> &mut HardwareContext {
        &mut self.context
    }

    pub async fn execute(&mut self, instructions: Vec<Instruction>) -> Result<()> {
        for instruction in instructions {
            let r = self.instruction(instruction).await;
            r?;
        }

        Ok(())
    }

    pub async fn record(&mut self, instructions: Vec<Instruction>) -> Result<Run> {
        if let Some(audio) = &self.audio {
            let _ = audio.queue(AudioFile::Up);
        }

        self.sink.start_recording().await;

        self.execute(instructions).await.inspect_err(|_| {
            if let Some(audio) = &self.audio {
                let _ = audio.queue(AudioFile::Double);
            }
        })?;

        let run = self.sink.stop_recording().await;

        if let Some(audio) = &self.audio {
            let _ = audio.queue(AudioFile::Down);
        }

        self.session.append_run(&run).await?;

        Ok(run)
    }

    async fn instruction(&mut self, instruction: Instruction) -> Result<()> {
        match instruction {
            Instruction::LoadCell(command) => {
                require_agent(&mut self.context.load_cell)?
                    .command(command)
                    .await?;
            }

            Instruction::Robot(command) => {
                require_agent(&mut self.context.robot_arm)?
                    .command(command)
                    .await?;
            }

            Instruction::Wind(command) => {
                require_agent(&mut self.context.wind_shape)?
                    .command(command)
                    .await?;
            }

            Instruction::BiasAll => {
                for device in self.context.iter_mut() {
                    device.bias().await;
                }
            }

            Instruction::Sleep(duration) => {
                sleep(duration).await;
            }

            Instruction::Reset => {
                for device in self.context.iter_mut() {
                    device.clear_error().await;
                }
            }
        }

        Ok(())
    }

    pub async fn new_experiment(&mut self, name: String) -> Result<()> {
        self.session.new_experiment(name).await
    }

    pub async fn save_experiment(&mut self) -> Result<()> {
        self.session.save_experiment().await
    }

    pub async fn new_run(&mut self) {
        self.sink.start_recording().await;
    }

    pub async fn save_run(&mut self) -> Result<Run> {
        let run = self.sink.stop_recording().await;

        self.session.append_run(&run).await?;

        Ok(run)
    }
}

/* == Misc == */

fn require_agent<T: HardwareAgent>(module: &mut Option<T>) -> Result<&mut T> {
    module
        .as_mut()
        .ok_or_else(|| eyre!("Hardware {} not available", type_name::<T>()))
}
