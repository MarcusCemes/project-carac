use std::{
    mem,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};

use bytes::{Buf, BufMut};
use tokio::{sync::Mutex, time::Instant};

const DEFAULT_CAPACITY: usize = 8388608; // 8MB

pub struct Stream {
    pub labels: Box<[String]>,
}

#[derive(Default)]
pub struct Recorder {
    inner: Arc<RecorderInner>,
    streams: Vec<Stream>,
}

pub struct RecordHandle<const N: usize> {
    inner: Arc<RecorderInner>,
    stream_id: u8,
}

#[derive(Default)]
struct RecorderInner {
    record: AtomicBool,
    tape: Mutex<Option<Tape>>,
}

impl Recorder {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn insert_tape(&mut self, tape: Option<Tape>) -> Option<Tape> {
        let mut guard = self.inner.tape.lock().await;
        mem::replace(&mut *guard, tape)
    }

    pub fn add_stream<S: Into<String>, const N: usize>(
        &mut self,
        labels: [S; N],
    ) -> RecordHandle<N> {
        let stream_id = self.streams.len() as u8;

        self.streams.push(Stream {
            labels: labels.map(|s| s.into()).into(),
        });

        RecordHandle {
            inner: self.inner.clone(),
            stream_id,
        }
    }

    pub fn set_recording(&self, record: bool) {
        self.inner.record.store(record, Ordering::SeqCst);
    }

    pub fn streams(&self) -> &[Stream] {
        &self.streams
    }
}

impl<const N: usize> RecordHandle<N> {
    pub async fn append(&self, data: [f32; N]) {
        if self.inner.record.load(Ordering::SeqCst) {
            let mut guard = self.inner.tape.lock().await;

            if let Some(tape) = &mut *guard {
                tape.append(self.stream_id, &data).await;
            }
        }
    }
}

/* == Tape == */

pub struct Tape {
    data: Vec<u8>,
    start: Instant,
}

impl Tape {
    pub fn new() -> Self {
        Self::with_capacity(DEFAULT_CAPACITY)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
            start: Instant::now(),
        }
    }

    pub async fn append(&mut self, stream_id: u8, data: &[f32]) {
        let buf = &mut self.data;

        buf.put_u64(self.start.elapsed().as_nanos() as u64);
        buf.put_u8(stream_id);

        for value in data {
            buf.put_f32(*value)
        }
    }

    pub fn reset(&mut self) {
        self.data.clear();
        self.start = Instant::now();
    }

    pub fn iter_data<'a>(
        &'a self,
        streams: &'a [Stream],
    ) -> impl Iterator<Item = (u64, u8, Box<[f32]>)> + 'a {
        let mut data = &self.data[..];

        std::iter::from_fn(move || {
            if data.is_empty() {
                return None;
            }

            let elapsed = data.get_u64();
            let stream_id = data.get_u8();

            let n_labels = streams[stream_id as usize].labels.len();
            let mut values = Vec::with_capacity(n_labels);

            for _ in 0..n_labels {
                values.push(data.get_f32());
            }

            Some((elapsed, stream_id, values.into_boxed_slice()))
        })
    }
}

impl Default for Tape {
    fn default() -> Self {
        Self::new()
    }
}
