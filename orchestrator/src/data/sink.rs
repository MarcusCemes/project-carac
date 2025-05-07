use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

use bytes::{Buf, BufMut};
use chunked_bytes::ChunkedBytes;
use eyre::{bail, Result};
use tokio::{sync::Mutex, time::Instant};

use crate::{
    data::run::{RecordedStream, Run, StreamInfo},
    misc::plot_juggler::PlotJugglerBroadcaster,
};

/* === Definitions === */

/* == Public == */

#[derive(Default)]
pub struct DataSink {
    inner: Arc<Inner>,
}

pub struct StreamWriter {
    index: usize,
    inner: Arc<Inner>,
}

/* == Private == */

#[derive(Default)]
struct Inner {
    recording: AtomicBool,
    shared: Mutex<InnerShared>,
}

#[derive(Default)]
struct InnerShared {
    buffers: Vec<Buffer>,
    plot_juggler: Option<PlotJugglerBroadcaster>,
    recording: bool,
    reference_time: Option<Instant>,
    streams: Vec<StreamInfo>,
}

#[derive(Default)]
struct Buffer(ChunkedBytes);

/* === Implementations === */

impl DataSink {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn add_stream(&self, name: String, channels: Vec<String>) -> Result<StreamWriter> {
        let mut lock = self.inner.shared.lock().await;

        if lock.streams.iter().any(|stream| stream.name == name) {
            bail!("Stream with name {} already exists", name);
        }

        let index = lock.buffers.len();
        let inner = self.inner.clone();

        lock.buffers.push(Buffer::default());
        lock.streams.push(StreamInfo { name, channels });

        Ok(StreamWriter { index, inner })
    }

    pub async fn clear(&self) {
        let mut lock = self.inner.shared.lock().await;

        for buffer in &mut lock.buffers {
            buffer.clear();
        }

        lock.reference_time = None;
    }

    pub async fn finish(&self) -> Run {
        let mut lock = self.inner.shared.lock().await;

        lock.reference_time = None;

        let data = lock.finish().collect();

        Run::new(data)
    }

    pub async fn set_record(&self, recording: bool) {
        self.inner.recording.store(recording, Ordering::SeqCst);
        self.inner.shared.lock().await.recording = recording;
    }

    pub async fn streams(&self) -> Vec<StreamInfo> {
        let lock = self.inner.shared.lock().await;
        lock.streams.clone()
    }
}

impl StreamWriter {
    pub async fn add(&self, measurements: &[f32]) {
        // Check the atomic boolean first to reduce lock contention
        if self.inner.recording() {
            let mut lock = self.inner.shared.lock().await;

            // The shared recording bool is the true source of truth
            if lock.recording {
                let time_us = lock.elapsed_us();
                lock.buffers[self.index].append(time_us, measurements);
            }
        }
    }
}

impl Inner {
    fn recording(&self) -> bool {
        self.recording.load(Ordering::SeqCst)
    }
}

impl InnerShared {
    fn elapsed_us(&mut self) -> u32 {
        match self.reference_time {
            Some(instant) => {
                let delta = instant.elapsed().as_micros();
                delta.try_into().expect("Time overflow")
            }

            None => {
                self.reference_time = Some(Instant::now());
                0
            }
        }
    }

    fn finish(&mut self) -> impl Iterator<Item = RecordedStream> + '_ {
        self.buffers
            .iter_mut()
            .zip(self.streams.iter())
            .map(|(buffer, stream)| buffer.finish(stream.channels.len()))
    }
}

impl Buffer {
    pub fn clear(&mut self) {
        self.0.advance(self.0.remaining());
    }

    pub fn append(&mut self, time_us: u32, channels: &[f32]) {
        self.0.put_u32_ne(time_us);

        for datum in channels {
            self.0.put_f32_ne(*datum);
        }
    }

    pub fn finish(&mut self, n_channels: usize) -> RecordedStream {
        let n_samples = self.0.remaining() / (1 + n_channels);

        let mut timestamps = Vec::with_capacity(n_samples);
        let mut channels = Vec::with_capacity(n_samples * n_channels);

        while let Ok(time) = self.0.try_get_u32_ne() {
            timestamps.push(time);

            for _ in 0..n_channels {
                channels.push(self.0.get_f32_ne());
            }
        }

        RecordedStream::new(timestamps, channels)
    }
}
