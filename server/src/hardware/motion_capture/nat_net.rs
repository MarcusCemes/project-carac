use bytes::{Buf, BufMut, TryGetError};
use eyre::{Result, eyre};
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use std::{fmt::Display, mem};

use crate::misc::buf::{BufExt, Decode, DecodeError, Encode};

/* === Definitions === */

type Vec3 = (f32, f32, f32);

pub struct NatNetV3_1;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Version([u8; 4]);

pub enum Request {
    Connect,
    ModelDefinitions,
}

pub enum Message {
    ServerInfo(ServerInfo),
    ModelDefinitions(ModelDefinitions),
    DataFrame(DataFrame),
}

pub struct ServerInfo {
    pub version: Version,
}

pub struct ModelDefinitions {
    pub rigid_bodies: Vec<RigidBodyDescription>,
}

pub enum DataSet {
    RigidBody(RigidBodyDescription),
    Other,
}

pub struct RigidBodyDescription {
    pub id: i32,
    pub name: String,
    pub parent_id: i32,
    pub offset: Vector3<f32>,
}

#[derive(Debug, Clone)]
pub struct DataFrame {
    pub frame_number: u32,
    pub rigid_bodies: Vec<RigidBodyData>,
    pub suffix: Option<DataFrameSuffix>,
}

#[derive(Debug, Clone)]
pub struct RigidBodyData {
    pub id: i32,
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
    pub mean_marker_error: f32,
    pub tracking_valid: bool,
}

#[derive(Debug, Clone)]
pub struct DataFrameSuffix {
    pub timecode: u32,
    pub timecode_sub: u32,
    pub timestamp: f64,
    pub stamp_camera_mid_exposure: u64,
    pub stamp_data_received: u64,
    pub stamp_transmit: u64,
    pub is_recording: bool,
}

struct Skipper;

/* === Implementations === */

impl NatNetV3_1 {
    pub const APP_NAME: &str = env!("CARGO_PKG_NAME");
    pub const APP_VERSION: Version = Version([2, 0, 0, 0]);
    pub const SUPPORTED_VERSION: Version = Version([3, 1, 0, 0]);
}

impl Request {
    const CONNECT_SIZE: usize = 273;
    const NAME_SIZE: usize = 256;
}

impl Encode for Request {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        match self {
            Request::Connect => {
                buf.put_u16_le(0x00); // NAT_CONNECT
                buf.put_u16_le(Request::CONNECT_SIZE as u16);

                let mut buf = buf.limit(Request::CONNECT_SIZE);

                let truncated_length = NatNetV3_1::APP_NAME.len().min(Self::NAME_SIZE);
                let name = &NatNetV3_1::APP_NAME.as_bytes()[..truncated_length];

                buf.put_slice(name);
                buf.put_bytes(0, Self::NAME_SIZE - truncated_length);

                buf.put_slice(&NatNetV3_1::APP_VERSION.0);
                buf.put_slice(&NatNetV3_1::SUPPORTED_VERSION.0);

                // sConnectionOptions
                buf.put_u8(0);
                buf.put_slice(&NatNetV3_1::SUPPORTED_VERSION.0);

                // reserved
                buf.put_u32_le(0);

                buf.fill();
            }

            Request::ModelDefinitions => {
                buf.put_u16_le(0x04); // NAT_REQUEST_MODELDEF
                buf.put_u16_le(0x00); // size
            }
        }
    }
}

impl Decode for Message {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let id = buf.try_get_u16_le()?;
        let size = buf.try_get_u16_le()? as usize;

        let mut buf = buf.take(size);

        let message = match id {
            0x1 => Message::ServerInfo(ServerInfo::decode(&mut buf)?), // NAT_SERVERINFO
            0x5 => Message::ModelDefinitions(ModelDefinitions::decode(&mut buf)?), // NAT_MODELDEF
            0x7 => Message::DataFrame(DataFrame::decode(&mut buf)?),   // NAT_FRAMEOFDATA
            id => Err(DecodeError::Unknown(eyre!("Unknown message ID: {id}")))?,
        };

        Ok(message)
    }
}

impl ServerInfo {
    const VERSION_OFFSET: usize = 0x104;
}

impl Display for Version {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}.{}", self.0[0], self.0[1], self.0[2], self.0[3])
    }
}

/* == ServerInfo == */

impl ServerInfo {
    pub fn validate_version(&self) -> Result<()> {
        (self.version == NatNetV3_1::SUPPORTED_VERSION)
            .then_some(())
            .ok_or_else(|| {
                eyre!(
                    "Unsupported NatNet version: {}. Expected: {}",
                    self.version,
                    NatNetV3_1::SUPPORTED_VERSION
                )
            })
    }
}

impl Decode for ServerInfo {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let mut version = Version([0; 4]);

        buf.advance(Self::VERSION_OFFSET);
        buf.try_copy_to_slice(&mut version.0)?;

        Ok(ServerInfo { version })
    }
}

/* == Description == */

impl Decode for ModelDefinitions {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let mut rigid_bodies = Vec::new();

        for _ in 0..buf.try_get_u32_le()? {
            match DataSet::decode(buf)? {
                DataSet::RigidBody(rb) => rigid_bodies.push(rb),
                DataSet::Other => continue,
            }
        }

        Ok(ModelDefinitions { rigid_bodies })
    }
}

impl Decode for DataSet {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        match buf.try_get_u32_le()? {
            0x0 => Skipper::marketset_description(buf)?,

            0x1 => {
                let rb = RigidBodyDescription::decode(buf)?;
                return Ok(DataSet::RigidBody(rb));
            }

            0x2 => Skipper::skeleton_description(buf)?,
            0x3 => Skipper::force_plate_description(buf)?,
            0x4 => Skipper::device_description(buf)?,
            0x5 => Skipper::camera_description(buf)?,

            id => Err(eyre!("Unknown dataset type: {id}"))?,
        }

        Ok(DataSet::Other)
    }
}

impl Decode for RigidBodyDescription {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let name = buf.try_read_cstring()?;
        let id = buf.try_get_i32_le()?;
        let parent_id = buf.try_get_i32_le()?;
        let offset = buf.try_get_vec3()?;

        // skip markers
        let n_markers = buf.try_get_u32_le()? as usize;
        let block_size = n_markers * 16; // pos (Vec3) + id (u32)

        buf.try_advance(block_size)?;

        Ok(RigidBodyDescription {
            id,
            name,
            parent_id,
            offset,
        })
    }
}

/* == DataFrame == */

impl Decode for DataFrame {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let frame_number = buf.try_get_u32_le()?;

        Skipper::markersets_data(buf)?;
        Skipper::legacy_marker_data(buf)?;

        let n_rigid_bodies = buf.try_get_u32_le()? as usize;
        let mut rigid_bodies = Vec::with_capacity(n_rigid_bodies);

        for _ in 0..n_rigid_bodies {
            rigid_bodies.push(RigidBodyData::decode(buf)?);
        }

        Skipper::skeletons_data(buf)?;
        Skipper::labelled_markers_data(buf)?;
        Skipper::force_plate_data(buf)?;
        Skipper::device_data(buf)?;

        // Not too bothered if the suffix is missing...
        let suffix = DataFrameSuffix::decode(buf).ok();

        Ok(DataFrame {
            frame_number,
            rigid_bodies,
            suffix,
        })
    }
}

impl Decode for RigidBodyData {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        Ok(RigidBodyData {
            id: buf.try_get_i32_le()?,
            position: buf.try_get_vec3()?,
            orientation: buf.try_get_quat()?,
            mean_marker_error: buf.try_get_f32_le()?,
            tracking_valid: (buf.try_get_i16_le()? & 0x01) != 0,
        })
    }
}

impl Decode for DataFrameSuffix {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError> {
        let timecode = buf.try_get_u32_le()?;
        let timecode_sub = buf.try_get_u32_le()?;
        let timestamp = buf.try_get_f64_le()?; // Double since v2.7
        let stamp_cam = buf.try_get_u64_le()?; // Since v3.0
        let stamp_recv = buf.try_get_u64_le()?; // Since v3.0
        let stamp_tx = buf.try_get_u64_le()?; // Since v3.0

        let params = buf.try_get_i16_le()?;
        let _eod = buf.try_get_u32_le()?; // End of data marker

        Ok(DataFrameSuffix {
            timecode,
            timecode_sub,
            timestamp,
            stamp_camera_mid_exposure: stamp_cam,
            stamp_data_received: stamp_recv,
            stamp_transmit: stamp_tx,
            is_recording: (params & 0x01) != 0,
        })
    }
}

/* == Skipper == */

impl Skipper {
    fn camera_description<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        let _name = buf.try_read_cstring()?;

        const BLOCK_SIZE: usize = 28; // Vec3 + Vec4
        buf.try_advance(BLOCK_SIZE)?;

        Ok(())
    }

    fn device_data<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        for _ in 0..buf.try_get_u32_le()? {
            let _id = buf.try_get_u32_le()?;

            for _ in 0..buf.try_get_u32_le()? {
                for _ in 0..buf.try_get_u32_le()? {
                    buf.try_get_f32_le()?;
                }
            }
        }

        Ok(())
    }

    fn device_description<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        let _id = buf.try_get_i32_le()?;
        let _name = buf.try_read_cstring()?;
        let _serial = buf.try_read_cstring()?;
        let _dev_type = buf.try_get_i32_le()?;
        let _channel_data_type = buf.try_get_i32_le()?;

        for _ in 0..buf.try_get_u32_le()? {
            let _channel_name = buf.try_read_cstring()?;
        }

        Ok(())
    }

    fn force_plate_data<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        for _ in 0..buf.try_get_u32_le()? {
            let _id = buf.try_get_u32_le()?;
            let n_channels = buf.try_get_u32_le()?;

            for _ in 0..n_channels {
                for _ in 0..buf.try_get_u32_le()? {
                    buf.try_get_f32_le()?;
                }
            }
        }

        Ok(())
    }

    fn force_plate_description<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        let _id = buf.try_get_i32_le()?;
        let _serial = buf.try_read_cstring()?;
        let _width = buf.try_get_f32_le()?;
        let _length = buf.try_get_f32_le()?;

        let _origin = buf.try_get_vec3()?;

        const BLOCK_SIZE: usize = 624; // calibration matrix + corners
        buf.try_advance(BLOCK_SIZE)?;

        let _plate_type = buf.try_get_i32_le()?;
        let _channel_data_type = buf.try_get_i32_le()?;

        for _ in 0..buf.try_get_u32_le()? {
            let _channel_name = buf.try_read_cstring()?;
        }

        Ok(())
    }

    fn labelled_markers_data<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        // ID (u32), pos (Vec3), size (u32), params (i16), residual (u32)
        const BLOCK_SIZE: usize = 26;

        let count = buf.try_get_u32_le()? as usize;
        buf.try_advance(BLOCK_SIZE * count)?;

        Ok(())
    }

    fn legacy_marker_data<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        let count = buf.try_get_u32_le()? as usize;
        let block_size = count * mem::size_of::<Vec3>();

        buf.try_advance(block_size)?;

        Ok(())
    }

    fn marketset_description<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        let _name = buf.try_read_cstring()?;

        for _ in 0..buf.try_get_u32_le()? {
            let _marker = buf.try_read_cstring()?;
        }

        Ok(())
    }

    fn markersets_data<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        for _ in 0..buf.try_get_u32_le()? {
            let _name = buf.try_read_cstring()?;

            let n_markers = buf.try_get_u32_le()? as usize;
            let block_size = n_markers * mem::size_of::<Vec3>();

            buf.try_advance(block_size)?;
        }

        Ok(())
    }

    fn skeleton_description<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        let _name = buf.try_read_cstring()?;
        let _id = buf.try_get_i32_le()?;

        for _ in 0..buf.try_get_u32_le()? {
            RigidBodyDescription::decode(buf)?; // skip
        }

        Ok(())
    }

    fn skeletons_data<B: Buf>(buf: &mut B) -> Result<(), DecodeError> {
        for _ in 0..buf.try_get_u32_le()? {
            let _id = buf.try_get_i32_le()?;
            let n_bones = buf.try_get_u32_le()? as usize;
            let block_size = n_bones * 38;

            buf.try_advance(block_size)?;
        }

        Ok(())
    }
}

/* == Extensions == */

trait NatNetExt: Buf {
    fn try_get_vec3(&mut self) -> Result<Vector3<f32>, DecodeError> {
        Ok(Vector3::new(
            self.try_get_f32_le()?,
            self.try_get_f32_le()?,
            self.try_get_f32_le()?,
        ))
    }

    fn try_get_quat(&mut self) -> Result<UnitQuaternion<f32>, DecodeError> {
        let parts = Vector4::new(
            self.try_get_f32_le()?, // x
            self.try_get_f32_le()?, // y
            self.try_get_f32_le()?, // z
            self.try_get_f32_le()?, // w
        );

        Ok(UnitQuaternion::new_normalize(Quaternion::from(parts)))
    }

    fn try_read_cstring(&mut self) -> Result<String, TryGetError> {
        let mut string = String::new();

        loop {
            match self.try_get_u8()? {
                0 => return Ok(string),
                char => string.push(char as char),
            }
        }
    }
}

impl<T: Buf> NatNetExt for T {}

trait NatNetMutExt: BufMut {
    fn fill(&mut self) {
        self.put_bytes(0, self.remaining_mut());
    }
}

impl<T: BufMut> NatNetMutExt for T {}
