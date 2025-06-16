use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum Command {
    SetFanSpeed(f32),
    SetPowered(bool),
    WaitSettled,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Status {
    pub in_control: bool,
}
