use bytes::{Buf, BufMut, TryGetError};
use eyre::Report;
use thiserror::Error;

/* == Encode/decode traits == */
pub trait Encode {
    fn encode<B: BufMut>(&self, buf: &mut B);
}

pub trait Decode: Sized {
    fn decode<B: Buf>(buf: &mut B) -> Result<Self, DecodeError>;
}

#[derive(Debug, Error)]
pub enum DecodeError {
    #[error("Not enough bytes available")]
    UnexpectedEndOfStream,

    #[error("{0}")]
    Unknown(Report),
}

impl From<Report> for DecodeError {
    fn from(err: Report) -> Self {
        DecodeError::Unknown(err)
    }
}

impl From<TryGetError> for DecodeError {
    fn from(_err: TryGetError) -> Self {
        DecodeError::UnexpectedEndOfStream
    }
}

/* == Trait extensions == */

pub trait BufExt: Buf {
    fn clear(&mut self) {
        self.advance(self.remaining());
    }

    fn ensure_capacity(&mut self, requested: usize) -> Result<(), TryGetError> {
        match self.remaining() {
            n if n >= requested => Ok(()),
            available => Err(TryGetError {
                available,
                requested,
            }),
        }
    }

    fn try_advance(&mut self, requested: usize) -> Result<(), TryGetError> {
        self.ensure_capacity(requested)?;
        self.advance(requested);
        Ok(())
    }

    fn try_get_bool(&mut self) -> Result<bool, TryGetError> {
        Ok(self.try_get_u8()? != 0)
    }

    fn try_get_string(&mut self) -> Result<String, TryGetError> {
        let len = self.try_get_u8()?;
        let mut bytes = vec![0; len as usize];

        self.try_copy_to_slice(&mut bytes)?;

        Ok(String::from_utf8(bytes).unwrap())
    }
}

impl<T: Buf> BufExt for T {}

pub trait BufMutExt: BufMut {
    fn ensure_capacity_mut(&mut self, requested: usize) -> Result<(), TryGetError> {
        match self.remaining_mut() {
            n if n >= requested => Ok(()),
            available => Err(TryGetError {
                available,
                requested,
            }),
        }
    }

    fn put_bool(&mut self, value: bool) {
        self.put_u8(if value { 1 } else { 0 });
    }

    fn put_string(&mut self, string: &str) {
        assert!(string.len() < u8::MAX as usize, "String too long");
        let bytes = string.as_bytes();
        self.put_u8(bytes.len() as u8);
        self.put_slice(bytes);
    }
}

impl<T: BufMut> BufMutExt for T {}
