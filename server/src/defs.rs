use bytes::{Buf, BufMut};
use nalgebra::{Isometry3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::misc::buf::{Decode, DecodeError, Encode};

pub const G: f32 = 9.80665;

/* == Points == */

/// Represents a point in 3D space with a position and an orientation, given as
/// Euler angles (roll, pitch, yaw) in radians.
#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub position: Vector3<f32>,
    pub orientation: Vector3<f32>,
}

impl Point {
    pub const CHANNELS: [&str; Self::WIDTH] = ["x", "y", "z", "roll", "pitch", "yaw"];
    pub const WIDTH: usize = 6;

    pub const ZERO: Self = Self {
        position: Vector3::new(0., 0., 0.),
        orientation: Vector3::new(0., 0., 0.),
    };

    pub const fn new(x: f32, y: f32, z: f32, rx: f32, ry: f32, rz: f32) -> Self {
        Self {
            position: Vector3::new(x, y, z),
            orientation: Vector3::new(rx, ry, rz),
        }
    }

    pub const fn position(x: f32, y: f32, z: f32) -> Self {
        Self {
            position: Vector3::new(x, y, z),
            orientation: Vector3::new(0., 0., 0.),
        }
    }

    pub fn quaternion(&self) -> UnitQuaternion<f32> {
        let o = &self.orientation;
        UnitQuaternion::from_euler_angles(o.x, o.y, o.z)
    }

    pub const fn array(&self) -> [f32; Self::WIDTH] {
        let [[x, y, z]] = self.position.data.0;
        let [[rx, ry, rz]] = self.orientation.data.0;

        [x, y, z, rx, ry, rz]
    }
}

impl Encode for Point {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        buf.put_f32(self.position.x);
        buf.put_f32(self.position.y);
        buf.put_f32(self.position.z);
        buf.put_f32(self.orientation.x);
        buf.put_f32(self.orientation.y);
        buf.put_f32(self.orientation.z);
    }
}

impl Decode for Point {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        Ok(Self {
            position: Vector3::new(buf.try_get_f32()?, buf.try_get_f32()?, buf.try_get_f32()?),
            orientation: Vector3::new(buf.try_get_f32()?, buf.try_get_f32()?, buf.try_get_f32()?),
        })
    }
}

impl From<&[f32; Self::WIDTH]> for Point {
    fn from(array: &[f32; Self::WIDTH]) -> Self {
        let [fx, fy, fz, ox, oy, oz] = *array;

        Point {
            position: Vector3::new(fx, fy, fz),
            orientation: Vector3::new(ox, oy, oz),
        }
    }
}

impl From<Point> for Isometry3<f32> {
    fn from(value: Point) -> Self {
        Isometry3::from_parts(value.position.into(), value.quaternion())
    }
}

impl From<Isometry3<f32>> for Point {
    fn from(value: Isometry3<f32>) -> Self {
        let position = value.translation.vector;
        let orientation = value.rotation.euler_angles();

        Point {
            position: Vector3::new(position.x, position.y, position.z),
            orientation: Vector3::new(orientation.0, orientation.1, orientation.2),
        }
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PointQ {
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
}

impl PointQ {
    pub fn new(position: Vector3<f32>, orientation: UnitQuaternion<f32>) -> Self {
        Self {
            position,
            orientation,
        }
    }
}

impl From<&Point> for PointQ {
    fn from(value: &Point) -> Self {
        let [[x, y, z]] = value.orientation.data.0;

        PointQ {
            position: value.position,
            orientation: UnitQuaternion::from_euler_angles(x, y, z),
        }
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Joints([f32; Self::WIDTH]);

impl Joints {
    const WIDTH: usize = 6;
}

impl Encode for Joints {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        for value in self.0 {
            buf.put_f32(value);
        }
    }
}

impl Decode for Joints {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let mut joints = [0.0; Self::WIDTH];

        for value in &mut joints {
            *value = buf.try_get_f32()?;
        }

        Ok(Self(joints))
    }
}

/* == Force/Moment ==  */

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Load {
    pub force: Vector3<f32>,
    pub moment: Vector3<f32>,
}

impl Load {
    pub const CHANNELS: [&str; Self::WIDTH] = ["fx", "fy", "fz", "mx", "my", "mz"];
    pub const WIDTH: usize = 6;

    pub const fn new(fx: f32, fy: f32, fz: f32, mx: f32, my: f32, mz: f32) -> Self {
        Self {
            force: Vector3::new(fx, fy, fz),
            moment: Vector3::new(mx, my, mz),
        }
    }

    pub const fn array(&self) -> [f32; Self::WIDTH] {
        let [[fx, fy, fz]] = self.force.data.0;
        let [[mx, my, mz]] = self.moment.data.0;
        [fx, fy, fz, mx, my, mz]
    }
}
