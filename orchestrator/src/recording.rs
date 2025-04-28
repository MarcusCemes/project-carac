use std::{
    io::{Read, Write},
    iter,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};

use bincode::{Decode, Encode};
use bytes::{Buf, BufMut};
use chrono::{DateTime, Utc};
use chunked_bytes::ChunkedBytes;
use eyre::{bail, Context, Result};
use tokio::{
    sync::{Mutex, MutexGuard},
    time::Instant,
};

use crate::{
    data::SegmentedRecordingIterator,
    misc::{compact_config, plot_juggler::PlotJugglerBroadcaster},
};

const MAGIC_NUMBER: u32 = u32::from_be_bytes(*b"RCDG");
const FORMAT_VERSION: u8 = 1;

#[derive(Default)]
pub struct Sink {
    inner: Arc<Inner>,
}

pub struct StreamWriter {
    definition: Arc<StreamDefinition>,
    inner: Arc<Inner>,
}

#[derive(Clone, Debug, Decode, Encode)]
pub struct StreamDefinition {
    pub name: String,
    pub channels: Vec<String>,
}

struct Stream {
    buffer: StreamBuffer,
    definition: Arc<StreamDefinition>,
}

#[derive(Default)]
struct Inner {
    recording: AtomicBool,
    shared: Mutex<InnerShared>,
}

struct InnerShared {
    plot_juggler: Option<PlotJugglerBroadcaster>,
    reference_instant: Instant,
    reference_time: DateTime<Utc>,
    streams: Vec<Stream>,
}

impl Sink {
    pub fn new() -> Self {
        Self::default()
    }

    /* == Streams == */

    pub async fn add_stream(&self, name: String, channels: Vec<String>) -> Result<StreamWriter> {
        let mut lock = self.inner.shared.lock().await;

        if lock.find_stream_with_name(&name).is_some() {
            bail!("Stream with name {} already exists", name);
        }

        let definition = Arc::new(StreamDefinition { name, channels });

        lock.streams.push(Stream::new(definition.clone()));

        Ok(StreamWriter {
            definition,
            inner: self.inner.clone(),
        })
    }

    pub async fn remove_stream(&self, name: &str) -> bool {
        let mut lock = self.inner.shared.lock().await;

        let maybe_stream = lock.find_stream_with_name(name);

        if let Some(i) = maybe_stream {
            lock.streams.swap_remove(i);
        }

        maybe_stream.is_some()
    }

    /* == Recording == */

    pub fn set_record(&self, active: bool) {
        self.inner.recording.store(active, Ordering::SeqCst);
    }

    pub async fn complete(&self) -> Recording {
        self.set_record(false);

        let mut lock = self.inner.shared.lock().await;
        tracing::debug!("Completed recording");

        let timestamp_us = lock.reference_time.timestamp_micros();
        let streams = lock.streams.iter_mut().map(Stream::complete).collect();

        Recording::new(timestamp_us, streams)
    }

    pub async fn clear_buffer(&self) {
        for stream in &mut self.inner.shared.lock().await.streams {
            stream.buffer.clear();
        }
    }

    pub async fn set_time_now(&self) {
        let mut lock = self.inner.shared.lock().await;
        lock.reference_instant = Instant::now();
        lock.reference_time = Utc::now();
    }

    pub async fn streams(&self) -> Vec<StreamDefinition> {
        let lock = self.inner.shared.lock().await;

        lock.streams
            .iter()
            .map(|s| (*s.definition).clone())
            .collect()
    }
}

impl StreamWriter {
    pub async fn write_now(&self, channels: &[f32]) {
        if self.inner.recording.load(Ordering::SeqCst) {
            let mut lock = self.inner.shared.lock().await;
            let time_us = lock.elapsed_micros();
            Self::write(&mut lock, &self.definition, time_us, channels)
        }
    }

    pub async fn write_at(&self, channels: &[f32], time_us: u32) {
        if self.inner.recording.load(Ordering::SeqCst) {
            Self::write(
                &mut self.inner.shared.lock().await,
                &self.definition,
                time_us,
                channels,
            );
        }
    }

    fn write(
        lock: &mut MutexGuard<'_, InnerShared>,
        definition: &Arc<StreamDefinition>,
        time_us: u32,
        channels: &[f32],
    ) {
        // Verify that the number of channels matches the stream definition
        debug_assert_eq!(channels.len(), definition.channels.len());

        let i = lock
            .find_stream_with_definition(definition)
            .expect("Stream not registered with sink!");

        lock.streams[i].buffer.store(time_us, channels);
    }
}

impl StreamDefinition {
    pub fn n_channels(&self) -> usize {
        self.channels.len()
    }

    pub fn qualified_channel_names(&self) -> impl Iterator<Item = String> + '_ {
        self.channels.iter().map(|s| format!("{}/{}", self.name, s))
    }
}

impl InnerShared {
    fn elapsed_micros(&self) -> u32 {
        let delta = self.reference_instant.elapsed();
        delta.as_micros().try_into().expect("Time overflow")
    }

    fn find_stream_with_name(&self, name: &str) -> Option<usize> {
        self.streams.iter().position(|s| s.definition.name == name)
    }

    fn find_stream_with_definition(&self, definition: &Arc<StreamDefinition>) -> Option<usize> {
        self.streams
            .iter()
            .position(|s| Arc::ptr_eq(&s.definition, definition))
    }
}

impl Default for InnerShared {
    fn default() -> Self {
        Self {
            plot_juggler: None,
            reference_instant: Instant::now(),
            reference_time: Utc::now(),
            streams: Vec::new(),
        }
    }
}

impl Stream {
    pub fn new(definition: Arc<StreamDefinition>) -> Self {
        Self {
            buffer: StreamBuffer::new(),
            definition,
        }
    }

    pub fn complete(&mut self) -> RecordedStream {
        let n_channels = self.definition.n_channels();

        // Create a data buffer to dump (unsorted) measurements into
        let mut unsorted_data = Vec::new();

        // Drain the buffer, appending data to buffer, collecting timestamps and indices
        let mut samples = iter::from_fn(|| self.buffer.retrieve(&mut unsorted_data, n_channels))
            .enumerate()
            .collect::<Vec<_>>();

        // Check that the buffer was completely exhausted
        debug_assert!(self.buffer.0.is_empty());

        // Sort the samples by their timestamp
        samples.sort_by_key(|(_, t)| *t);

        let mut data = Vec::with_capacity(samples.len() * n_channels);

        // Use the sorted samples to append measurements to the data buffer
        for (i, _) in &samples {
            let start = i * n_channels;
            let end = start + n_channels;
            data.extend_from_slice(&unsorted_data[start..end]);
        }

        let data_timestamps_us = samples.into_iter().map(|(_, t)| t).collect();
        let definition = (*self.definition).clone();

        RecordedStream {
            data,
            data_timestamps_us,
            definition,
        }
    }
}

/* == StreamBuffer == */

struct StreamBuffer(ChunkedBytes);

impl StreamBuffer {
    pub fn new() -> Self {
        Self(ChunkedBytes::new())
    }

    pub fn store(&mut self, time_us: u32, channels: &[f32]) {
        let buffer = &mut self.0;
        buffer.put_u32_ne(time_us);
        channels.iter().for_each(|v| buffer.put_f32_ne(*v));
    }

    pub fn retrieve(&mut self, data: &mut Vec<f32>, n_channels: usize) -> Option<u32> {
        let time = self.0.try_get_u32_ne().ok()?;
        (0..n_channels).for_each(|_| data.push(self.0.get_f32_ne()));
        Some(time)
    }

    pub fn clear(&mut self) {
        self.0.advance(self.0.remaining());
    }
}

/* == Recording == */

#[derive(Clone, Decode, Encode)]
pub struct Recording {
    pub start_timestamp_us: i64,
    pub recorded_streams: Vec<RecordedStream>,
}

#[derive(Clone, Decode, Encode)]
pub struct RecordedStream {
    pub definition: StreamDefinition,
    pub data_timestamps_us: Vec<u32>,
    pub data: Vec<f32>,
}

#[derive(Copy, Clone, Debug)]
pub struct RecordedSample<'a> {
    pub channel_data: &'a [f32],
    pub definition: &'a StreamDefinition,
    pub recording_timestamp_us: u32,
}

impl Recording {
    pub fn new(start_timestamp_us: i64, streams: Vec<RecordedStream>) -> Self {
        Self {
            start_timestamp_us,
            recorded_streams: streams,
        }
    }

    pub fn segment(&self, divisions: u32) -> Option<SegmentedRecordingIterator<'_>> {
        SegmentedRecordingIterator::new(self, divisions)
    }

    /* == Encoding & decoding == */

    pub fn decode<R: Read>(reader: &mut R) -> Result<Self> {
        bincode::decode_from_std_read(reader, compact_config())
            .wrap_err("Failed to decode recording")
    }

    pub fn encode<W: Write>(&self, writer: &mut W) -> Result<()> {
        bincode::encode_into_std_write(self, writer, compact_config())
            .wrap_err("Failed to encode recording")?;

        Ok(())
    }
}

impl RecordedStream {
    pub fn iter_samples(&self) -> impl Iterator<Item = RecordedSample<'_>> {
        let n_channels = self.definition.n_channels();

        let timestamps = self.data_timestamps_us.iter();
        let data = self.data.chunks_exact(self.definition.n_channels());

        // Check that the data length matches the expected size
        debug_assert_eq!(self.data.len(), timestamps.len() * n_channels);

        timestamps
            .zip(data)
            .map(move |(&recording_timestamp, channel_data)| RecordedSample {
                channel_data,
                definition: &self.definition,
                recording_timestamp_us: recording_timestamp,
            })
    }
}

impl RecordedSample<'_> {
    pub fn timestamp_s(&self) -> f32 {
        1e-6 * self.recording_timestamp_us as f32
    }
}
