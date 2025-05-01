use bytes::{Buf, BufMut};

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
