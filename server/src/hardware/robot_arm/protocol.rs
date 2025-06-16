use bytes::{Buf, BufMut};
use eyre::eyre;

use crate::misc::buf::{BufExt, BufMutExt, Decode, DecodeError, Encode};

use super::defs::*;

const MAGIC_HEADER: u8 = 0x95;

pub struct Request {
    command: Command,
    header: Header,
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
    Settled(bool),
}

/* === Implementations === */

impl Request {
    pub fn new(request_id: u8, command: Command) -> Self {
        Self {
            command,
            header: Header::new(request_id),
        }
    }
}

impl Encode for Request {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        buf.put_u8(self.header.magic);
        buf.put_u8(self.header.request_id);

        self.command.encode(buf);
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

impl Encode for Command {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        match self {
            Command::Hello => {
                buf.put_u8(0x00);
            }

            Command::Halt(return_home) => {
                buf.put_u8(0x01);
                buf.put_bool(*return_home);
            }

            Command::SetPowered(powered) => {
                buf.put_u8(0x02);
                buf.put_bool(*powered);
            }

            Command::ReturnHome(motion_type) => {
                buf.put_u8(0x03);
                buf.put_u8(*motion_type as u8);
            }

            Command::SetReporting(enabled) => {
                buf.put_u8(0x04);
                buf.put_bool(*enabled);
            }

            Command::SetInterval(interval_s) => {
                buf.put_u8(0x05);
                buf.put_f32(*interval_s);
            }

            Command::SetConfig(config) => {
                buf.put_u8(0x06);
                config.encode(buf);
            }

            Command::SetProfile(profile) => {
                buf.put_u8(0x07);
                profile.encode(buf);
            }

            Command::SetBlending(blending) => {
                buf.put_u8(0x08);
                buf.put_u8(blending.kind as u8);
                buf.put_f32(blending.reach);
                buf.put_f32(blending.leave);
            }

            Command::SetToolOffset(offset) => {
                buf.put_u8(0x09);
                offset.encode(buf);
            }

            Command::ResetMoveId => {
                buf.put_u8(0x0A);
            }

            Command::Move(motion) => match motion {
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

            _ => unimplemented!("Not a remote command!"),
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
            0x84 => Response::Settled(buf.try_get_bool()?),

            code => {
                let report = eyre!("Unsupported code: 0x{code:02x}");
                tracing::error!("{report}");

                return Err(DecodeError::from(report));
            }
        };

        Ok(response)
    }
}
