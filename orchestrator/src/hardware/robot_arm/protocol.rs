use bytes::{Buf, BufMut};
use eyre::eyre;

use crate::misc::buf::{BufMutExt, Decode, DecodeError, Encode};

use super::defs::*;

const MAGIC_HEADER: u8 = 0x95;

pub struct Request {
    header: Header,
    instruction: Instruction,
}

struct Header {
    magic: u8,
    request_id: u8,
}

/* === Definitions ===  */

#[derive(Debug)]
pub enum Response {
    Ack(u8),
    Error(u16),
    PowerOff,
    State(State),
}

/* === Implementations === */

impl Request {
    pub fn new(request_id: u8, instruction: Instruction) -> Self {
        Self {
            header: Header::new(request_id),
            instruction,
        }
    }
}

impl Encode for Request {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        buf.put_u8(self.header.magic);
        buf.put_u8(self.header.request_id);

        self.instruction.encode(buf);
    }
}

impl Header {
    pub fn new(request_id: u8) -> Self {
        Self {
            magic: MAGIC_HEADER,
            request_id,
        }
    }
}

impl Encode for Instruction {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        match self {
            Instruction::Hello => {
                buf.put_u8(0x00);
            }

            Instruction::Halt(return_home) => {
                buf.put_u8(0x01);
                buf.put_bool(*return_home);
            }

            Instruction::SetPowered(powered) => {
                buf.put_u8(0x02);
                buf.put_bool(*powered);
            }

            Instruction::ReturnHome(motion_type) => {
                buf.put_u8(0x03);
                buf.put_u8(*motion_type as u8);
            }

            Instruction::SetReporting(enabled) => {
                buf.put_u8(0x04);
                buf.put_bool(*enabled);
            }

            Instruction::SetFrequency(interval_s) => {
                buf.put_u8(0x05);
                buf.put_f32(*interval_s);
            }

            Instruction::SetConfig(config) => {
                buf.put_u8(0x06);
                config.encode(buf);
            }

            Instruction::SetProfile(profile) => {
                buf.put_u8(0x07);
                profile.encode(buf);
            }

            Instruction::SetBlending(blending) => {
                buf.put_u8(0x08);
                buf.put_u8(blending.kind as u8);
                buf.put_f32(blending.reach);
                buf.put_f32(blending.leave);
            }

            Instruction::SetToolOffset(offset) => {
                buf.put_u8(0x09);
                offset.encode(buf);
            }

            Instruction::Move(motion) => match motion {
                Motion::Direct(point) => {
                    buf.put_u8(0x20);
                    point.encode(buf);
                }

                Motion::Linear(point) => {
                    buf.put_u8(0x21);
                    point.encode(buf);
                }

                Motion::Joint(joints) => {
                    buf.put_u8(0x22);
                    joints.encode(buf);
                }

                Motion::Circular((a, b)) => {
                    buf.put_u8(0x23);
                    a.encode(buf);
                    b.encode(buf);
                }
            },
        }
    }
}

impl Decode for Response {
    fn decode<B: Buf>(buf: &mut B) -> Result<Response, DecodeError> {
        if buf.try_get_u8()? != MAGIC_HEADER {
            Err(eyre!("Missing magic header"))?;
        }

        let response = match buf.try_get_u8()? {
            0x00 => {
                panic!("Attempted to decode handshake, may be picking up loopback traffic!");
            }

            0x80 => Response::Ack(buf.try_get_u8()?),
            0x81 => Response::Error(buf.try_get_u16()?),
            0x82 => Response::PowerOff,
            0x83 => Response::State(State::decode(buf)?),

            code => {
                let report = eyre!("Unsupported code: {code:02x}");
                tracing::error!("{report}");

                return Err(DecodeError::from(report));
            }
        };

        Ok(response)
    }
}
