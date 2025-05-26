use bytes::{Buf, BufMut};
use eyre::Result;
use serde::{Deserialize, Serialize};
use strum::EnumDiscriminants;

use crate::{
    defs::*,
    misc::buf::{BufExt, Decode, DecodeError, Encode},
};

/* == ArmConfig == */

#[derive(Debug, Clone, Copy, Default, Deserialize, Serialize)]
pub struct ArmConfig {
    shoulder: ArmConfigKind,
    elbow: ArmConfigKind,
    wrist: ArmConfigKind,
}

#[derive(Debug, Clone, Copy, Default, Deserialize, Serialize)]
enum ArmConfigKind {
    // Do not modify order
    #[default]
    Free,
    Same,
    RightlyPositive,
    LeftNegative,
}

/* == Blending == */

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct BlendingConfig {
    pub kind: BlendingKind,
    pub leave: f32,
    pub reach: f32,
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub enum BlendingKind {
    // Do not modify order
    Off,
    Joint,
    Cartesian,
}

/* == Command == */

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(untagged)]
pub enum Command {
    WaitSettled,
    SetOrigin(Point),
    Remote(Instruction),
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub enum Instruction {
    Halt(bool),
    Hello,
    Move(Motion),
    ReturnHome(MotionKind),
    SetBlending(BlendingConfig),
    SetConfig(ArmConfig),
    SetFrequency(f32),
    SetPowered(bool),
    SetProfile(Profile),
    SetReporting(bool),
    SetToolOffset(Point),
}

/* == Motion == */

#[derive(Copy, Clone, Debug, Deserialize, Serialize, EnumDiscriminants)]
#[strum_discriminants(derive(Deserialize, Serialize), name(MotionKind))]
pub enum Motion {
    Direct(Point),
    Linear(Point),
    Joint(Joints),
    Circular((Point, Point)),
}

/* == Profile == */

#[derive(Copy, Clone, Debug, Default, Deserialize, Serialize)]
pub struct Profile {
    pub limit: ProfileLimit,
    pub scale: ProfileScale,
}

#[derive(Default)]
pub struct ProfileBuilder(Profile);

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct ProfileLimit {
    pub translation: f32,
    pub rotation: f32,
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct ProfileScale {
    pub acceleration: u8,
    pub velocity: u8,
    pub deceleration: u8,
}

/* == State == */

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct State {
    pub position: Point,
    pub joints: Joints,
    pub joint_error: Joints,
    pub settled: bool,
}

/* == Implementations === */

impl Encode for ArmConfig {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        for v in [self.shoulder, self.elbow, self.wrist] {
            buf.put_u8(v as u8);
        }
    }
}

impl Encode for Motion {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        match self {
            Self::Linear(point) | Self::Direct(point) => point.encode(buf),
            Self::Joint(joints) => joints.encode(buf),

            Self::Circular((start, end)) => {
                start.encode(buf);
                end.encode(buf);
            }
        }
    }
}

impl Profile {
    pub fn builder() -> ProfileBuilder {
        ProfileBuilder(Profile::default())
    }
}

impl Encode for Profile {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        buf.put_f32(self.limit.translation);
        buf.put_f32(self.limit.rotation);
        buf.put_u8(self.scale.acceleration);
        buf.put_u8(self.scale.velocity);
        buf.put_u8(self.scale.deceleration);
    }
}

impl ProfileBuilder {
    pub fn build(self) -> Profile {
        self.0
    }

    pub fn with_translation(mut self, value: f32) -> Self {
        self.0.limit.translation = value;
        self
    }

    pub fn with_rotation(mut self, value: f32) -> Self {
        self.0.limit.rotation = value;
        self
    }

    pub fn with_smoothing(mut self, value: u8) -> Self {
        self.0.scale.acceleration = value;
        self.0.scale.deceleration = value;
        self
    }
}

impl Default for ProfileLimit {
    fn default() -> Self {
        Self {
            translation: 10000.,
            rotation: 10000.,
        }
    }
}

impl Default for ProfileScale {
    fn default() -> Self {
        Self {
            acceleration: 100,
            velocity: 100,
            deceleration: 100,
        }
    }
}

impl Decode for State {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        Ok(Self {
            position: Point::decode(buf)?,
            joints: Joints::decode(buf)?,
            joint_error: Joints::decode(buf)?,
            settled: buf.try_get_bool()?,
        })
    }
}
