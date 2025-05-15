use std::time::Duration;

use eyre::{ContextCompat, Result, bail};
use serde::{Deserialize, Serialize};
use tokio::time::sleep;

use crate::{
    config::Config,
    data::{experiment::Run, session::Session, sink::DataSink},
    hardware::{
        HardwareAgent, HardwareContext, load_cell::LoadCellInstruction,
        robot_arm::RobotArmInstruction, wind_shape::WindShapeInstruction,
    },
    misc::{plot_juggler::PlotJugglerBroadcaster, type_name},
};

pub struct Orchestrator {
    context: HardwareContext,
    session: Session,
    sink: DataSink,
}

impl Orchestrator {
    pub async fn create(config: Config, mut context: HardwareContext) -> Result<Self> {
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

        let session = Session::open(path, streams).await?;

        Ok(Self {
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

    pub async fn execute(&mut self, instructions: Vec<Instruction>) -> Result<Run> {
        self.sink.start_recording().await;

        for instruction in instructions {
            self.execute_instruction(instruction).await?;
        }

        let run = self.sink.stop_recording().await;

        self.session.append_run(&run).await?;

        Ok(run)
    }

    async fn execute_instruction(&mut self, instruction: Instruction) -> Result<()> {
        match instruction {
            Instruction::LoadCell(instruction) => {
                require_module(&mut self.context.load_cell)?
                    .execute(instruction)
                    .await?;
            }

            Instruction::RobotArm(instruction) => {
                require_module(&mut self.context.robot_arm)?
                    .execute(instruction)
                    .await?;
            }

            Instruction::WindShape(instruction) => {
                require_module(&mut self.context.wind_shape)?
                    .execute(instruction)
                    .await?;
            }

            Instruction::Sleep(duration) => {
                sleep(duration).await;
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

/* == Instruction == */

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum Instruction {
    LoadCell(LoadCellInstruction),
    RobotArm(RobotArmInstruction),
    WindShape(WindShapeInstruction),
    Sleep(Duration),
}

/* == Misc == */

fn require_module<T: HardwareAgent>(module: &mut Option<T>) -> Result<&mut T> {
    match module {
        Some(module) => Ok(module),
        None => bail!("Module {} not available", type_name::<T>()),
    }
}
