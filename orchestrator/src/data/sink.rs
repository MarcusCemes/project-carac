use std::sync::Arc;

use bytes::{Buf, BufMut};
use chunked_bytes::ChunkedBytes;
use serde::{Deserialize, Serialize};
use tokio::{
    sync::{Mutex, RwLock},
    time::Instant,
};

use crate::{
    data::experiment::{RecordedStream, Run},
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

#[derive(Clone, Debug, Serialize, Deserialize)]
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

struct Buffer(ChunkedBytes, usize);

/* === Implementations === */

impl DataSinkBuilder {
    pub async fn with_context(mut self, context: &mut HardwareContext) -> Self {
        for agent in context.iter_mut() {
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

        buffer_lock.push(Buffer(ChunkedBytes::new(), stream.channels.len()));
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
        let lock = self.inner.shared.write().await;
        let mut buffer_lock = lock.buffers.lock().await;

        for buffer in &mut *buffer_lock {
            buffer.clear();
        }
    }

    pub async fn start_recording(&self) {
        tracing::info!("Recording started");
        let mut lock = self.inner.shared.write().await;

        lock.reference_time = Some(Instant::now());
    }

    pub async fn stop_recording(&self) -> Run {
        tracing::info!("Recording stopped");

        // Acquire exclusive access to stop recording
        self.inner.shared.write().await.reference_time = None;

        // Reacquire the lock in read mode to allow for concurrent access
        let lock = self.inner.shared.read().await;
        let mut buffer_lock = lock.buffers.lock().await;

        // Read-out the buffers and create a new run
        let recorded_streams = buffer_lock.iter_mut().map(Buffer::finish).collect();

        Run::new(recorded_streams)
    }

    pub async fn set_broadcaster(&self, plot_juggler: Option<PlotJugglerBroadcaster>) {
        let msg = match plot_juggler {
            Some(_) => "Broadcaster enabled",
            None => "Broadcaster disabled",
        };

        tracing::info!("{}", msg);
        self.inner.shared.write().await.broadcaster = plot_juggler.map(Broadcaster::new);
    }
}

impl StreamWriter {
    pub async fn add(&self, channel_data: &[f32]) {
        let lock = self.inner.shared.read().await;

        // Broadcast irrespective of recording state
        if let Some(broadcaster) = &lock.broadcaster {
            let _ = broadcaster.plot_juggler.send(
                broadcaster.reference_time.elapsed().as_secs_f32(),
                channel_data,
                &self.stream,
            );
        }

        // Record the data is a time reference is set
        if let Some(time_us) = lock.elapsed_us() {
            let mut buffer_lock = lock.buffers.lock().await;
            buffer_lock[self.index].append(time_us, channel_data);
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

    pub fn use_or(streams: Option<&[StreamInfo]>, channels: &[u8]) -> Vec<StreamInfo> {
        match streams {
            Some(streams) => Vec::from(streams),

            None => channels
                .iter()
                .map(|&channel| StreamInfo {
                    name: format!("stream_{:2}", channel),
                    channels: vec![format!("channel_{:2}", channel)],
                })
                .collect(),
        }
    }
}

impl Shared {
    fn elapsed_us(&self) -> Option<u32> {
        self.reference_time.map(|instant| {
            instant
                .elapsed()
                .as_micros()
                .try_into()
                .expect("Time overflow")
        })
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

    pub fn finish(&mut self) -> RecordedStream {
        let n_samples = self.0.remaining() / (1 + self.1);

        let mut timestamps = Vec::with_capacity(n_samples);
        let mut channels = Vec::with_capacity(n_samples * self.1);

        while let Ok(time) = self.0.try_get_u32_ne() {
            timestamps.push(time);

            for _ in 0..self.1 {
                channels.push(self.0.get_f32_ne());
            }
        }

        RecordedStream::new(timestamps, channels)
    }
}
