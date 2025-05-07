use std::{mem, time::Duration};

use eyre::{bail, ContextCompat, Result};
use serde::{Deserialize, Serialize};
use tokio::time::sleep;

use crate::{
    config::Config,
    data::{
        experiment::{Experiment, ExperimentManager, ExperimentMetadata},
        run::Run,
        sink::DataSink,
    },
    hardware::{
        load_cell::LoadCellInstruction, robot_arm::RobotArmInstruction,
        wind_shape::WindShapeInstruction, HardwareContext,
    },
    misc::type_name,
};

pub struct Orchestrator {
    context: HardwareContext,
    manager: ExperimentManager,
    runs: Vec<Run>,
    sink: DataSink,
}

impl Orchestrator {
    pub async fn create(config: Config, context: HardwareContext) -> Result<Self> {
        let path = config.sink.session_path.wrap_err("Session path not set")?;

        let manager = ExperimentManager::create(path).await?;
        let sink = DataSink::new();

        Ok(Self {
            context,
            manager,
            runs: Vec::new(),
            sink,
        })
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

    pub async fn finish_experiment(&mut self, name: Option<String>) -> Result<Experiment> {
        let experiment = Experiment::new(
            ExperimentMetadata::new(name, self.sink.streams().await),
            mem::take(&mut self.runs),
        );

        self.manager.save_experiment(&experiment).await?;

        Ok(experiment)
    }

    pub async fn finish_run(&mut self) {
        let run = self.sink.finish().await;
        self.runs.push(run);
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

fn require_module<T>(module: &mut Option<T>) -> Result<&mut T> {
    match module {
        Some(module) => Ok(module),
        None => bail!("Module {} not enabled", type_name::<T>()),
    }
}
