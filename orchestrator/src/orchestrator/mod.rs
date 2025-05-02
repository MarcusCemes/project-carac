use std::time::Duration;

use eyre::{bail, eyre, Context, Result};
use serde::{Deserialize, Serialize};
use tokio::time::sleep;

use crate::{
    defs::Point,
    hardware::{
        robot_arm::{Motion, RobotCommand, SpeedProfile},
        HardwareContext,
    },
    misc::type_name,
    recording::Sink,
};

pub struct Orchestrator {
    context: HardwareContext,
    sink: Sink,
}

impl Orchestrator {
    pub fn new(context: HardwareContext) -> Self {
        let sink = Sink::new();
        Self { context, sink }
    }

    pub async fn execute(&mut self, instruction: Instruction) -> Result<()> {
        match instruction {
            Instruction::LoadCell(instruction) => {
                use LoadCellInstruction as I;

                let load_cell = require_module(&mut self.context.load_cell)?;

                let result = match instruction {
                    I::SetBias => load_cell.set_bias().await,
                    I::SetRecord(true) => load_cell.start_streaming().await,
                    I::SetRecord(false) => load_cell.stop_streaming().await,
                };

                result.wrap_err("Load cell error")
            }

            Instruction::MotionCapture(instruction) => {
                use MotionCaptureInstruction as I;

                let motion_capture = require_module(&mut self.context.motion_capture)?;

                match instruction {
                    I::Subscribe(marker) => motion_capture.subscribe(&marker, &self.sink).await,
                    I::Unsubscribe(marker) => {
                        motion_capture.unsubscribe(&marker, &self.sink).await;
                        Ok(())
                    }
                }
            }

            Instruction::RobotArm(instruction) => {
                use RobotArmInstruction as I;
                use RobotCommand as C;

                let robot_arm = require_module(&mut self.context.robot_arm)?;

                let command = match instruction {
                    I::Move(motion) => C::Move(motion),
                    I::SetOffset(offset) => C::SetToolOffset(offset),
                    I::SetProfile(profile) => C::SetSpeedProfile(profile),
                    I::WaitSettled => {
                        return robot_arm
                            .try_wait_settled()
                            .await
                            .map_err(|code| eyre!("Robot error (code {code}"))
                    }
                };

                robot_arm.execute_command(command).await
            }

            Instruction::WindShape(instruction) => {
                use WindShapeInstruction as I;

                let wind_shape = require_module(&mut self.context.wind_shape)?;

                match instruction {
                    I::EnablePower(true) => wind_shape.enable_power().await,
                    I::EnablePower(false) => wind_shape.disable_power().await,
                    I::ReleaseControl => Ok(wind_shape.release_control().await),
                    I::RequestControl => Ok(wind_shape.request_control().await),
                    I::SetWindSpeed(speed) => wind_shape.set_fan_speed(speed).await,
                }
            }

            Instruction::AddMarker(name) => Ok(self.sink.add_marker(name).await),
            Instruction::CreateSpan(name) => self.sink.create_span(&name).await,
            Instruction::NewRecording => Ok(self.sink.clear_buffer().await),
            Instruction::SetRecord(record) => Ok(self.sink.set_record(record)),
            Instruction::ResetTime => Ok(self.sink.set_time_now().await),
            Instruction::Sleep(duration) => Ok(sleep(duration).await),

            Instruction::SaveRecording(_name) => todo!(),
        }
    }
}

/* == Instruction == */

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub enum Instruction {
    AddMarker(String),
    CreateSpan(String),
    LoadCell(LoadCellInstruction),
    MotionCapture(MotionCaptureInstruction),
    NewRecording,
    ResetTime,
    RobotArm(RobotArmInstruction),
    SetRecord(bool),
    Sleep(Duration),
    WindShape(WindShapeInstruction),
    SaveRecording(String),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub enum MotionCaptureInstruction {
    Subscribe(String),
    Unsubscribe(String),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub enum LoadCellInstruction {
    SetBias,
    SetRecord(bool),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub enum RobotArmInstruction {
    Move(Motion),
    SetOffset(Point),
    SetProfile(SpeedProfile),
    WaitSettled,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub enum WindShapeInstruction {
    EnablePower(bool),
    ReleaseControl,
    RequestControl,
    SetWindSpeed(u8),
}

/* == Misc == */

fn require_module<T>(module: &mut Option<T>) -> Result<&mut T> {
    match module {
        Some(module) => Ok(module),
        None => bail!("Module {} not enabled", type_name::<T>()),
    }
}
