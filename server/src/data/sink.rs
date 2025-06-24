use std::sync::Arc;

use bytes::{Buf, BufMut};
use chunked_bytes::ChunkedBytes;
use serde::{Deserialize, Serialize};
use tokio::{
    sync::{Mutex, RwLock},
    time::Instant,
};

use crate::{
    data::experiment::{RecordedStream, Run, SampleTime},
    hardware::HardwareContext,
    misc::plot_juggler::PlotJugglerBroadcaster,
};

/* === Definitions === */

/* == Public == */

#[derive(Default)]
pub struct DataSinkBuilder {
    inner: Arc<Inner>,
    streams: Vec<StreamInfo>,
}

pub struct DataSink {
    inner: Arc<Inner>,
}

pub struct StreamWriter {
    index: usize,
    inner: Arc<Inner>,
    stream: StreamInfo,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct StreamInfo {
    pub name: String,
    pub channels: Vec<String>,
}

/* == Private == */

#[derive(Default)]
struct Inner {
    shared: RwLock<Shared>,
}

#[derive(Default)]
struct Shared {
    buffers: Mutex<Vec<Buffer>>,
    broadcaster: Option<Broadcaster>,
    reference_time: Option<Instant>,
}

struct Broadcaster {
    plot_juggler: PlotJugglerBroadcaster,
    reference_time: Instant,
}

impl Broadcaster {
    pub fn new(plot_juggler: PlotJugglerBroadcaster) -> Self {
        Self {
            plot_juggler,
            reference_time: Instant::now(),
        }
    }
}

struct Buffer {
    buf: ChunkedBytes,
    n_channels: usize,
}

/* === Implementations === */

impl DataSinkBuilder {
    pub async fn with_context(mut self, context: &mut HardwareContext) -> Self {
        for agent in context.iter() {
            agent.register(&mut self).await;
        }

        self
    }

    pub fn build(self) -> (DataSink, Vec<StreamInfo>) {
        tracing::debug!("Building DataSink");

        let sink = DataSink { inner: self.inner };

        (sink, self.streams)
    }

    pub async fn register_stream(&mut self, name: String, channels: Vec<String>) -> StreamWriter {
        tracing::debug!("Registering stream {name}");

        if self.streams.iter().any(|stream| stream.name == name) {
            panic!("Stream with name {name} already exists");
        }

        let inner = self.inner.clone();

        let lock = self.inner.shared.read().await;
        let mut buffer_lock = lock.buffers.lock().await;

        let index = buffer_lock.len();
        let stream = StreamInfo { name, channels };

        let buffer = Buffer {
            buf: ChunkedBytes::new(),
            n_channels: stream.channels.len(),
        };

        buffer_lock.push(buffer);
        self.streams.push(stream.clone());

        StreamWriter {
            index,
            inner,
            stream,
        }
    }
}

impl DataSink {
    pub fn builder() -> DataSinkBuilder {
        DataSinkBuilder::default()
    }

    pub async fn clear_buffers(&self) {
        let lock = self.inner.shared.read().await;
        let mut buffer_lock = lock.buffers.lock().await;

        for buffer in &mut *buffer_lock {
            buffer.clear();
        }
    }

    pub async fn start_recording(&self) {
        tracing::debug!("Starting recording");
        let mut lock = self.inner.shared.write().await;

        lock.reference_time = Some(Instant::now());
    }

    pub async fn stop_recording(&self) -> Run {
        tracing::debug!("Stopping recording");

        // Acquire exclusive access to stop recording
        self.inner.shared.write().await.reference_time = None;

        // Reacquire the lock in read mode to allow for concurrent access
        let lock = self.inner.shared.read().await;
        let mut buffer_lock = lock.buffers.lock().await;

        // Read-out the buffers and create a new run
        let recorded_streams = buffer_lock.iter_mut().map(Buffer::finish).collect();

        Run::new(recorded_streams)
    }

    pub async fn set_broadcaster(&self, plot_juggler: PlotJugglerBroadcaster) {
        self.inner.shared.write().await.broadcaster = Some(Broadcaster::new(plot_juggler));
    }

    pub async fn clear_broadcaster(&self) {
        self.inner.shared.write().await.broadcaster = None;
    }
}

impl StreamWriter {
    pub async fn add(&self, instant: Instant, channel_data: &[f32]) {
        debug_assert_eq!(channel_data.len(), self.stream.channels.len());

        let lock = self.inner.shared.read().await;

        // Broadcast irrespective of recording state
        if let Some(broadcaster) = &lock.broadcaster {
            let _ = broadcaster.plot_juggler.send(
                broadcaster.reference_time.elapsed().as_secs_f32(),
                channel_data,
                &self.stream,
            );
        }

        // Record the data if a time reference is set
        if let Some(time_us) = lock.elapsed_us_from(instant) {
            let mut buffer_lock = lock.buffers.lock().await;
            buffer_lock[self.index].append(time_us, channel_data);
        }
    }

    pub async fn add_many(&self, instant: Instant, frequency: f32, channel_data: &[f32]) {
        debug_assert!(frequency > 0.);
        debug_assert!(channel_data.len() >= self.stream.channels.len());
        debug_assert_eq!(channel_data.len() % self.stream.channels.len(), 0);

        let period = 1e6 / frequency;
        let n_channels = self.stream.channels.len();
        let n_samples = channel_data.len() / n_channels;
        let duration_us = n_samples as f32 * period;

        let lock = self.inner.shared.read().await;

        // Send only the last sample
        if let Some(broadcaster) = &lock.broadcaster {
            let i = channel_data.len() - n_channels;

            let _ = broadcaster.plot_juggler.send(
                broadcaster.reference_time.elapsed().as_secs_f32(),
                &channel_data[i..],
                &self.stream,
            );
        }

        // Record the data if a time reference is set
        if let Some(time_us) = lock.elapsed_us_from(instant) {
            let (start_us, offset, duration_us) = match time_us.checked_sub(duration_us as u32) {
                Some(start_us) => (start_us, 0, duration_us),

                // If the range exceeds the zero timestamp, trim some starting samples
                None => {
                    let k_samples = (time_us as f32 / period) as usize;
                    let offset = n_samples - k_samples;
                    let start_us = time_us - ((k_samples - 1) as f32 * period) as u32;
                    let duration_us = k_samples as f32 * period;

                    (start_us, offset, duration_us)
                }
            };

            let mut buffer_lock = lock.buffers.lock().await;

            for i in offset..n_samples {
                let start = i * n_channels;
                let end = start + n_channels;
                let progress = (i - offset) as f32 / (n_samples - offset) as f32;
                let time = start_us + (duration_us * progress) as u32;

                buffer_lock[self.index].append(time, &channel_data[start..end]);
            }
        }
    }
}

impl StreamInfo {
    pub fn qualified_channel_names(&self) -> Vec<String> {
        self.channels
            .iter()
            .map(|channel| format!("{}/{}", self.name, channel))
            .collect()
    }
}

impl Shared {
    fn elapsed_us_from(&self, instant: Instant) -> Option<u32> {
        self.reference_time.map(|ref_instant| {
            instant
                .duration_since(ref_instant)
                .as_micros()
                .try_into()
                .expect("Time overflow")
        })
    }
}

impl Buffer {
    pub fn clear(&mut self) {
        self.buf.advance(self.buf.remaining());
    }

    pub fn append(&mut self, time_us: u32, channels: &[f32]) {
        self.buf.put_u32_ne(time_us);

        for datum in channels {
            self.buf.put_f32_ne(*datum);
        }
    }

    pub fn finish(&mut self) -> RecordedStream {
        let n_samples = self.buf.remaining() / (1 + self.n_channels);

        let mut timestamps = Vec::with_capacity(n_samples);
        let mut channels = Vec::with_capacity(n_samples * self.n_channels);

        while let Ok(time) = self.buf.try_get_u32_ne() {
            timestamps.push(SampleTime(time));

            for _ in 0..self.n_channels {
                channels.push(self.buf.get_f32_ne());
            }
        }

        // Check that all timestamps are in increasing order
        debug_assert!(
            timestamps.windows(2).all(|w| w[0] < w[1]),
            "Timestamps not recorded in increasing order"
        );

        RecordedStream::new(timestamps, channels, self.n_channels)
    }
}
