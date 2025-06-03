use std::net::{IpAddr, Ipv4Addr};

use bytes::{Buf, BufMut};
use eyre::{Result, eyre};
use tokio::{io, net::UdpSocket};

use crate::misc::buf::{DecodeError, Encode};

/* == Definitions ==  */

pub struct Link {
    socket: UdpSocket,
}

struct Request<'a> {
    state: &'a [f32],
}

pub struct Update {
    pub state: Vec<f32>,
}

/* == Implementations ==  */

impl Link {
    pub const BUFFER_SIZE: usize = 1024;
    pub const MAGIC_NUMBER: u8 = 0xDE;

    pub async fn try_new(ip: IpAddr, port: u16) -> Result<Self> {
        let socket = UdpSocket::bind((IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0)).await?;
        let addr = (ip, port);

        socket.connect(addr).await?;

        Ok(Link { socket })
    }

    pub async fn send_state(&self, state: &[f32], buf: &mut Vec<u8>) -> io::Result<()> {
        buf.clear();

        Request { state }.encode(buf);
        self.socket.send(&buf).await?;

        Ok(())
    }

    pub async fn receive_state(&self, data: &mut Vec<f32>, buf: &mut Vec<u8>) -> Result<()> {
        self.socket.recv_buf(buf).await?;
        Self::decode_response(data, &mut &buf[..])?;

        Ok(())
    }

    fn decode_response<B: Buf>(data: &mut Vec<f32>, buf: &mut B) -> Result<(), DecodeError> {
        if buf.try_get_u8()? != Link::MAGIC_NUMBER {
            return Err(DecodeError::Unknown(eyre!("Missing magic header!")));
        }

        let count = buf.try_get_u8()? as usize;

        for _ in 0..count {
            data.push(buf.try_get_f32()?);
        }

        Ok(())
    }
}

impl Encode for Request<'_> {
    fn encode<B: BufMut>(&self, buf: &mut B) {
        buf.put_u8(Link::MAGIC_NUMBER);
        buf.put_u8(self.state.len() as u8);

        for &value in self.state {
            buf.put_f32(value);
        }
    }
}
