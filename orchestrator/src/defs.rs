use bincode::{Decode, Encode};
use nalgebra::{UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Pose {
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
}

impl Pose {
    pub const CHANNELS: [&str; 7] = ["x", "y", "z", "i", "j", "k", "w"];

    pub fn array(self) -> [f32; 7] {
        let (p, o) = (self.position, self.orientation);
        [p.x, p.y, p.z, o.i, o.j, o.k, o.w]
    }
}

#[derive(Copy, Clone, Debug, Decode, Default, Deserialize, Encode, PartialEq, Serialize)]
pub struct PoseEuler {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rx: f32,
    pub ry: f32,
    pub rz: f32,
}

impl PoseEuler {
    pub const fn new(x: f32, y: f32, z: f32, rx: f32, ry: f32, rz: f32) -> Self {
        Self {
            x,
            y,
            z,
            rx,
            ry,
            rz,
        }
    }

    pub fn array(self) -> [f32; 6] {
        [self.x, self.y, self.z, self.rx, self.ry, self.rz]
    }
}

impl From<&PoseEuler> for Pose {
    fn from(point: &PoseEuler) -> Self {
        Pose {
            position: Vector3::new(point.x, point.y, point.z),
            orientation: UnitQuaternion::from_euler_angles(point.rx, point.ry, point.rz),
        }
    }
}

#[derive(Copy, Clone, Debug, Decode, Default, Deserialize, Encode, PartialEq, Serialize)]
pub struct RobotJoints([f32; 6]);

/* == Load ==  */

#[derive(Copy, Clone, Debug)]
pub struct Load {
    pub force: Vector3<f32>,
    pub moment: Vector3<f32>,
}

impl Load {
    pub fn new(force: Vector3<f32>, moment: Vector3<f32>) -> Self {
        Self { force, moment }
    }

    pub fn array(&self) -> [f32; 6] {
        let [[fx, fy, fz]] = self.force.data.0;
        let [[tx, ty, tz]] = self.moment.data.0;
        [fx, fy, fz, tx, ty, tz]
    }
}

/* == Tests == */

#[cfg(test)]
mod tests {
    use std::mem;

    use super::*;

    const POSE_SIZE: usize = 6 * std::mem::size_of::<f32>();

    /// Verify sizes for bincode encode/decode
    #[test]
    fn test_struct_sizes() {
        assert_eq!(mem::size_of::<PoseEuler>(), POSE_SIZE);
        assert_eq!(mem::size_of::<RobotJoints>(), POSE_SIZE);
    }
}
