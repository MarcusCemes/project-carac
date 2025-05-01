use bytes::{Buf, BufMut, Bytes, TryGetError};
use std::{fmt, io, mem, string::FromUtf8Error};

const NAME: &[u8] = b"Orchestrator";
const VERSION: [u8; 4] = [1, 0, 0, 0];
const NATNET_VERSION: [u8; 4] = [3, 1, 0, 0];

pub type Vec3 = (f32, f32, f32);
pub type Vec4 = (f32, f32, f32, f32);

// --- NatNet Message IDs (consistent with previous definitions) ---
pub const NAT_CONNECT: u16 = 0;
pub const NAT_SERVERINFO: u16 = 1;
pub const NAT_REQUEST: u16 = 2;
pub const NAT_RESPONSE: u16 = 3;
pub const NAT_REQUEST_MODELDEF: u16 = 4;
pub const NAT_MODELDEF: u16 = 5;
pub const NAT_REQUEST_FRAMEOFDATA: u16 = 6;
pub const NAT_FRAMEOFDATA: u16 = 7;
pub const NAT_MESSAGESTRING: u16 = 8;
pub const NAT_DISCONNECT: u16 = 9;
pub const NAT_KEEPALIVE: u16 = 10;
pub const NAT_UNRECOGNIZED_REQUEST: u16 = 100;

// --- Custom Error Type ---
#[derive(Debug)]
pub enum ParseError {
    IoError(io::Error),
    UnexpectedEof,
    InvalidString(FromUtf8Error),
    UnknownDescriptionType(u32),
    Other(String),
}

impl From<TryGetError> for ParseError {
    fn from(_: TryGetError) -> Self {
        ParseError::UnexpectedEof
    }
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::IoError(e) => write!(f, "IO Error: {e}"),
            ParseError::UnexpectedEof => write!(f, "Unexpected end of buffer"),
            ParseError::InvalidString(e) => write!(f, "Invalid UTF-8 string: {e}"),
            ParseError::UnknownDescriptionType(t) => write!(f, "Unknown description type: {t}"),
            ParseError::Other(s) => write!(f, "Parse Error: {s}"),
        }
    }
}

impl std::error::Error for ParseError {}

impl From<io::Error> for ParseError {
    fn from(err: io::Error) -> Self {
        ParseError::IoError(err)
    }
}

impl From<FromUtf8Error> for ParseError {
    fn from(err: FromUtf8Error) -> Self {
        ParseError::InvalidString(err)
    }
}

fn try_advance<B: Buf>(buf: &mut B, size: usize) -> Result<(), ParseError> {
    if buf.remaining() < size {
        return Err(ParseError::UnexpectedEof);
    }

    buf.advance(size);
    Ok(())
}

fn try_read_bytes<B: Buf>(buf: &mut B, size: usize) -> Result<Bytes, ParseError> {
    let mut data = vec![0; size];
    buf.try_copy_to_slice(data.as_mut_slice())?;
    Ok(Bytes::from(data))
}

fn try_read_string<B: Buf>(buf: &mut B) -> Result<String, ParseError> {
    let mut string = String::new();

    loop {
        let char = buf.try_get_u8().map_err(|_| ParseError::UnexpectedEof)?;

        if char == 0 {
            return Ok(string);
        }

        string.push(char as char);
    }
}

fn try_read_vec3<B: Buf>(buf: &mut B) -> Result<Vec3, ParseError> {
    let x = buf.try_get_f32_le()?;
    let y = buf.try_get_f32_le()?;
    let z = buf.try_get_f32_le()?;

    Ok((x, y, z))
}

fn try_read_vec4<B: Buf>(buf: &mut B) -> Result<Vec4, ParseError> {
    let x = buf.try_get_f32_le()?;
    let y = buf.try_get_f32_le()?;
    let z = buf.try_get_f32_le()?;
    let w = buf.try_get_f32_le()?;

    Ok((x, y, z, w))
}

// --- Data Structures (Rust Mirrors of Python Dataclasses) ---
#[derive(Debug, Clone)]
pub struct Message {
    pub id: u16,
    pub payload: Bytes,
}

impl Message {
    pub fn packet(self) -> Bytes {
        let mut buf = Vec::with_capacity(self.payload.len() + 4);

        buf.put_u16_le(self.id);
        buf.put_u16_le(self.payload.len() as u16);
        buf.put(&*self.payload);

        Bytes::from(buf)
    }
}

#[derive(Debug, Clone)]
pub struct RigidBodyDesc {
    pub id: i32,
    pub name: String,
    pub parent_id: i32,
    pub offset: Vec3,
}

#[derive(Debug, Clone, Default)]
pub struct Description {
    pub rigid_bodies: Vec<RigidBodyDesc>,
}

impl Description {
    pub fn get_rb(&self, id: i32) -> Option<&RigidBodyDesc> {
        self.rigid_bodies.iter().find(|rb| rb.id == id)
    }

    pub fn get_rb_name(&self, name: &str) -> Option<&RigidBodyDesc> {
        self.rigid_bodies.iter().find(|rb| rb.name == name)
    }
}

#[derive(Debug, Clone)]
pub struct RigidBodyData {
    pub id: i32,
    pub position: Vec3,
    pub orientation: Vec4,
    pub mean_marker_error: f32,
    pub params: i16, // tracking valid = (params & 0x01) != 0
}

#[derive(Debug, Clone)]
pub struct FrameSuffix {
    pub timecode: u32,
    pub timecode_sub: u32,
    pub timestamp: f64,
    pub stamp_camera_mid_exposure: u64,
    pub stamp_data_received: u64,
    pub stamp_transmit: u64,
    pub params: i16, // is_recording = (params & 0x01) != 0, etc.
                     // eod marker read but not stored
}

#[derive(Debug, Clone)]
pub struct DataFrame {
    pub frame_number: u32,
    pub rigid_bodies: Vec<RigidBodyData>,
    pub suffix: Option<FrameSuffix>,
}

// --- Top-Level Message Parser ---

pub fn parse_message<B: Buf>(mut buf: B) -> Result<Message, ParseError> {
    let buf = &mut buf;

    let id = buf.try_get_u16_le()?;
    let size = buf.try_get_u16_le()?;
    let payload = try_read_bytes(buf, size as usize)?;

    Ok(Message { id, payload })
}

// --- Description Parsing (NAT_MODELDEF - NatNet v3.1) ---

pub fn parse_description<B: Buf>(mut buf: B) -> Result<Description, ParseError> {
    let mut buf = &mut buf;

    let mut rigid_bodies = Vec::new();

    let n_datasets = buf.try_get_u32_le()?;

    for _ in 0..n_datasets {
        if let Some(rigid_body) = parse_dataset(&mut buf)? {
            rigid_bodies.push(rigid_body);
        }
    }

    Ok(Description { rigid_bodies })
}

fn parse_dataset<B: Buf>(buf: &mut B) -> Result<Option<RigidBodyDesc>, ParseError> {
    let t = buf.try_get_u32_le()?;

    match t {
        0x0 => skip_marketset_description(buf)?,
        0x1 => return Ok(Some(parse_rigid_body_description(buf)?)),
        0x2 => skip_skeleton_description(buf)?,
        0x3 => skip_force_plate_description(buf)?,
        0x4 => skip_device_description(buf)?,
        0x5 => skip_camera_description(buf)?,
        id => panic!("Unknown dataset type: {id}"),
    }

    Ok(None)
}

fn skip_marketset_description<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let _name = try_read_string(buf)?;

    for _ in 0..buf.try_get_u32_le()? {
        let _marker = try_read_string(buf)?;
    }

    Ok(())
}

fn parse_rigid_body_description<B: Buf>(buf: &mut B) -> Result<RigidBodyDesc, ParseError> {
    let name = try_read_string(buf)?;

    let id = buf.try_get_i32_le()?;
    let parent_id = buf.try_get_i32_le()?;

    let offset = (
        buf.try_get_f32_le()?,
        buf.try_get_f32_le()?,
        buf.try_get_f32_le()?,
    );

    // skip markers
    let n_markers = buf.try_get_u32_le()?;
    let block_size = n_markers as usize * mem::size_of::<(Vec3, u32)>();

    try_advance(buf, block_size)?;

    Ok(RigidBodyDesc {
        id,
        name,
        parent_id,
        offset,
    })
}

fn skip_skeleton_description<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let _name = try_read_string(buf)?;
    let _id = buf.try_get_i32_le()?;

    for _ in 0..buf.try_get_u32_le()? {
        parse_rigid_body_description(buf)?;
    }

    Ok(())
}

fn skip_camera_description<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let _name = try_read_string(buf)?;

    const BLOCK_SIZE: usize = 28; // Vec3 + Vec4
    try_advance(buf, BLOCK_SIZE)?;

    Ok(())
}

fn skip_force_plate_description<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let _id = buf.try_get_i32_le()?;
    let _serial = try_read_string(buf)?;
    let _width = buf.try_get_f32_le()?;
    let _length = buf.try_get_f32_le()?;

    let _origin = (
        buf.try_get_f32_le()?,
        buf.try_get_f32_le()?,
        buf.try_get_f32_le()?,
    );

    const BLOCK_SIZE: usize = 624; // calibration matrix + corners
    try_advance(buf, BLOCK_SIZE)?;

    let _plate_type = buf.try_get_i32_le()?;
    let _channel_data_type = buf.try_get_i32_le()?;

    for _ in 0..buf.try_get_u32_le()? {
        let _channel_name = try_read_string(buf)?;
    }

    Ok(())
}

fn skip_device_description<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let _id = buf.try_get_i32_le()?;
    let _name = try_read_string(buf)?;
    let _serial = try_read_string(buf)?;
    let _dev_type = buf.try_get_i32_le()?;
    let _channel_data_type = buf.try_get_i32_le()?;

    for _ in 0..buf.try_get_u32_le()? {
        let _channel_name = try_read_string(buf)?;
    }

    Ok(())
}

// --- Frame Data Parsing (NAT_FRAMEOFDATA - NatNet v3.1) ---

/// Parses the payload of a NAT_FRAMEOFDATA message (v3.1 format).
pub fn parse_frame_data<B: Buf>(mut buf: B) -> Result<DataFrame, ParseError> {
    let buf = &mut buf;
    let frame_number = buf.try_get_u32_le()?;

    skip_markersets(buf)?;
    skip_legacy_markers(buf)?;

    let rigid_bodies = parse_rigid_body_data(buf)?;

    skip_skeleton_data(buf)?;
    skip_labelled_markers(buf)?;
    skip_force_plates(buf)?;
    skip_device_data(buf)?;

    let suffix = parse_frame_suffix(buf).ok();

    Ok(DataFrame {
        frame_number,
        rigid_bodies,
        suffix,
    })
}

fn skip_markersets<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let count = buf.try_get_u32_le()?;

    for _ in 0..count {
        let _name = try_read_string(buf)?;

        let marker_count = buf.try_get_u32_le()?;
        let block_size = marker_count as usize * mem::size_of::<Vec3>();
        try_advance(buf, block_size)?;
    }

    Ok(())
}

fn skip_legacy_markers<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let count = buf.try_get_u32_le()?;
    let block_size = count as usize * mem::size_of::<Vec3>();
    try_advance(buf, block_size)?;

    Ok(())
}

fn parse_rigid_body_data<B: Buf>(buf: &mut B) -> Result<Vec<RigidBodyData>, ParseError> {
    let count = buf.try_get_u32_le()?;

    let mut rigid_bodies = Vec::with_capacity(count as usize);

    for _ in 0..count {
        rigid_bodies.push(RigidBodyData {
            id: buf.try_get_i32_le()?,
            position: try_read_vec3(buf)?,
            orientation: try_read_vec4(buf)?,
            mean_marker_error: buf.try_get_f32_le()?,
            params: buf.try_get_i16_le()?,
        });
    }

    Ok(rigid_bodies)
}

fn skip_skeleton_data<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    for _ in 0..buf.try_get_u32_le()? {
        let _id = buf.try_get_i32_le()?;
        let bones = buf.try_get_u32_le()?;
        const BLOCK_SIZE: usize = 38;
        try_advance(buf, BLOCK_SIZE * bones as usize)?;
    }

    Ok(())
}

fn skip_labelled_markers<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    // ID (u32), pos (Vec3), size (u32), params (i16), residual (u32)
    const BLOCK_SIZE: usize = 26;

    let count = buf.try_get_u32_le()?;
    try_advance(buf, BLOCK_SIZE * count as usize)?;

    Ok(())
}

fn skip_force_plates<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    let count = buf.try_get_u32_le()?;

    for _ in 0..count {
        let _id = buf.try_get_u32_le()?;
        let n_channels = buf.try_get_u32_le()?;

        for _ in 0..n_channels {
            for _ in 0..buf.try_get_u32_le()? {
                try_advance(buf, mem::size_of::<f32>())?;
            }
        }
    }

    Ok(())
}

fn skip_device_data<B: Buf>(buf: &mut B) -> Result<(), ParseError> {
    for _ in 0..buf.try_get_u32_le()? {
        let _id = buf.try_get_u32_le()?;

        for _ in 0..buf.try_get_u32_le()? {
            for _ in 0..buf.try_get_u32_le()? {
                try_advance(buf, mem::size_of::<f32>())?;
            }
        }
    }

    Ok(())
}

fn parse_frame_suffix<B: Buf>(buf: &mut B) -> Result<FrameSuffix, ParseError> {
    let timecode = buf.try_get_u32_le()?;
    let timecode_sub = buf.try_get_u32_le()?;
    let timestamp = buf.try_get_f64_le()?; // Double since v2.7
    let stamp_cam = buf.try_get_u64_le()?; // Since v3.0
    let stamp_recv = buf.try_get_u64_le()?; // Since v3.0
    let stamp_tx = buf.try_get_u64_le()?; // Since v3.0

    let params = buf.try_get_i16_le()?;
    let _eod = buf.try_get_u32_le()?; // End of data marker

    Ok(FrameSuffix {
        timecode,
        timecode_sub,
        timestamp,
        stamp_camera_mid_exposure: stamp_cam,
        stamp_data_received: stamp_recv,
        stamp_transmit: stamp_tx,
        params,
    })
}

/* == Client messages == */

pub fn handshake_msg() -> Message {
    let mut buf = Vec::with_capacity(273);

    // client name (256 bytes)
    buf.put(NAME);
    buf.put_bytes(0, 256 - NAME.len());

    buf.put(VERSION.as_ref());
    buf.put(NATNET_VERSION.as_ref());

    debug_assert_eq!(buf.len(), 264);

    // sConnectionOptions
    buf.put_u8(0);
    buf.put(NATNET_VERSION.as_ref());

    // reserved
    buf.put_u32(0);

    Message {
        id: NAT_CONNECT,
        payload: Bytes::from(buf),
    }
}

pub fn request_definitions_msg() -> Message {
    Message {
        id: NAT_REQUEST_MODELDEF,
        payload: Bytes::new(),
    }
}
