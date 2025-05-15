use bytes::{Buf, BufMut};
use tokio::io::{self, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub trait BufExt {
    fn try_get_string(&mut self) -> Option<String>;
}

impl<T: Buf> BufExt for T {
    fn try_get_string(&mut self) -> Option<String> {
        let len = self.try_get_u8().ok()?;
        let mut bytes = vec![0; len as usize];

        self.copy_to_slice(&mut bytes);

        String::from_utf8(bytes).ok()
    }
}

pub trait BufMutExt {
    fn put_string(&mut self, string: &str);
}

impl<T: BufMut> BufMutExt for T {
    fn put_string(&mut self, string: &str) {
        let bytes = string.as_bytes();
        self.put_u8(bytes.len() as u8);
        self.put_slice(bytes);
    }
}

pub(crate) trait ReadExt {
    async fn read_string(&mut self) -> io::Result<String>;
}

pub(crate) trait WriteExt {
    async fn write_string(&mut self, string: &str) -> io::Result<()>;
}

impl<T: AsyncRead + Unpin> ReadExt for T {
    async fn read_string(&mut self) -> io::Result<String> {
        let len = self.read_u8().await? as usize;
        let mut bytes = vec![0; len];

        self.read_exact(&mut bytes).await?;

        Ok(String::from_utf8(bytes).unwrap())
    }
}

impl<T: AsyncWrite + Unpin> WriteExt for T {
    async fn write_string(&mut self, string: &str) -> io::Result<()> {
        let bytes = string.as_bytes();
        self.write_u8(bytes.len() as u8).await?;
        self.write_all(bytes).await?;
        Ok(())
    }
}
