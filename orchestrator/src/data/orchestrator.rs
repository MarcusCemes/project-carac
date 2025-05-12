use std::time::Duration;

use eyre::{ContextCompat, Result, bail};
use serde::{Deserialize, Serialize};
use tokio::time::sleep;

use crate::{
    config::Config,
    data::{session::Session, sink::DataSink},
    hardware::{
        HardwareAgent, HardwareContext, load_cell::LoadCellInstruction,
        robot_arm::RobotArmInstruction, wind_shape::WindShapeInstruction,
    },
    misc::type_name,
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
        let session = Session::create(path, streams).await?;

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

    pub async fn execute(&mut self, instruction: Instruction) -> Result<()> {
        match instruction {
            Instruction::LoadCell(instruction) => {
                require_module(&mut self.context.load_cell)?
                    .execute(instruction)
                    .await
            }

            Instruction::RobotArm(instruction) => {
                require_module(&mut self.context.robot_arm)?
                    .execute(instruction)
                    .await
            }

            Instruction::WindShape(instruction) => {
                require_module(&mut self.context.wind_shape)?
                    .execute(instruction)
                    .await
            }

            Instruction::Sleep(duration) => {
                sleep(duration).await;
                Ok(())
            }
        }
    }

    pub async fn create_experiment(&mut self, name: String) -> Result<()> {
        self.session.create_experiment(name).await
    }

    pub async fn finish_experiment(&mut self) {
        self.session.close_experiment();
    }

    pub async fn start_run(&mut self) {
        self.sink.start_recording().await;
    }

    pub async fn finish_run(&mut self) -> Result<()> {
        let run = self.sink.stop_recording().await;
        self.session.add_run(&run).await
    }
}

/* == Instruction == */

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
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
