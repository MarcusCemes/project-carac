use bincode::{Decode, Encode};
use quaternion::{Quaternion, euler_angles};
use serde::{Deserialize, Serialize};

pub type Vec3<T> = [T; 3];

#[derive(Copy, Clone, Debug, Default, PartialEq, Encode, Decode)]
pub struct Pose {
    pub position: Vec3<f32>,
    pub orientation: Quaternion<f32>,
}

impl Pose {
    pub fn to_array(self) -> [f32; 7] {
        let mut array = [0.; 7];
        array[0..3].copy_from_slice(&self.position);
        array[3] = self.orientation.0;
        array[4..7].copy_from_slice(&self.orientation.1);
        array
    }
}

// The required size for a Point or Joint
pub const POINT_JOINT_SIZE: usize = 6 * std::mem::size_of::<f32>();

#[derive(Copy, Clone, Debug, Decode, Default, Deserialize, Encode, PartialEq, Serialize)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rx: f32,
    pub ry: f32,
    pub rz: f32,
}

impl Point {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        rx: 0.0,
        ry: 0.0,
        rz: 0.0,
    };
}

impl From<&Point> for Pose {
    fn from(point: &Point) -> Self {
        Pose {
            position: [point.x, point.y, point.z],
            orientation: euler_angles(point.rx, point.ry, point.rz),
        }
    }
}

#[derive(Copy, Clone, Debug, Decode, Default, Deserialize, Encode, PartialEq, Serialize)]
pub struct Joint([f32; 6]);

#[cfg(test)]
mod tests {
    use std::mem;

    use super::*;

    #[test]
    fn motion_size() {
        assert_eq!(mem::size_of::<Point>(), POINT_JOINT_SIZE);
        assert_eq!(mem::size_of::<Joint>(), POINT_JOINT_SIZE);
    }
}
