use std::{
    io::{self, Read, Write},
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
use tokio::{sync::Mutex, time::Instant};

use crate::{data::SegmentedRecordingIterator, misc::compact_config};

const MAGIC_NUMBER: u32 = u32::from_be_bytes(*b"RCDG");
const FORMAT_VERSION: u8 = 1;

#[derive(Default)]
pub struct Recorder {
    inner: Arc<RecorderInner>,
}

#[derive(Default)]
struct RecorderInner {
    recording: AtomicBool,
    shared: Mutex<RecorderShared>,
}

struct RecorderShared {
    ref_time: Instant,
    start_time: DateTime<Utc>,
    streams: Vec<RecorderStream>,
}

struct RecorderStream {
    buffer: RecorderBuffer,
    definition: StreamDefinition,
}

#[derive(Clone, Debug, Decode, Encode)]
pub struct StreamDefinition {
    pub name: String,
    pub channels: Vec<String>,
}

pub struct StreamHandle {
    recorder: Arc<RecorderInner>,
    stream_id: usize,
}

impl Recorder {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn add_stream<T, U>(&self, name: T, channels: &[U]) -> StreamHandle
    where
        T: ToString,
        U: ToString,
    {
        let mut lock = self.inner.shared.lock().await;

        if lock.streams.len() >= u8::MAX as usize {
            panic!("Maximum number of streams reached");
        }

        let channels = channels.iter().map(ToString::to_string).collect();
        let name = name.to_string();

        let stream = StreamDefinition { name, channels };

        // This must be before pushing the stream to the vector!
        let stream_id = lock.streams.len();

        lock.streams.push(RecorderStream {
            buffer: RecorderBuffer::new(),
            definition: stream.clone(),
        });

        let recorder = self.inner.clone();

        StreamHandle {
            recorder,
            stream_id,
        }
    }

    pub fn start_recording(&self) {
        tracing::debug!("Resumed recording");
        self.inner.recording.store(true, Ordering::SeqCst);
    }

    pub fn stop_recording(&self) {
        tracing::debug!("Stopped recording");
        self.inner.recording.store(false, Ordering::SeqCst);
    }

    pub async fn complete(&self) -> Recording {
        self.stop_recording();

        let mut lock = self.inner.shared.lock().await;
        tracing::debug!("Completed recording");

        let timestamp_us = lock.start_time.timestamp_micros();

        let streams = lock
            .streams
            .iter_mut()
            .map(RecorderStream::complete)
            .collect();

        Recording::new(timestamp_us, streams)
    }

    pub async fn clear_buffer(&self) {
        tracing::debug!("Cleared recording buffer");

        for stream in &mut self.inner.shared.lock().await.streams {
            stream.buffer.clear();
        }
    }

    pub async fn reset_reference_time(&self) {
        tracing::debug!("Recording reference time reset");
        self.inner.shared.lock().await.ref_time = Instant::now();
        self.inner.shared.lock().await.start_time = Utc::now();
    }

    pub async fn streams(&self) -> Vec<StreamDefinition> {
        let lock = self.inner.shared.lock().await;
        lock.streams.iter().map(|s| s.definition.clone()).collect()
    }
}

impl RecorderShared {
    fn elapsed_micros(&self) -> u32 {
        self.ref_time
            .elapsed()
            .as_micros()
            .try_into()
            .expect("Time overflow")
    }
}

impl Default for RecorderShared {
    fn default() -> Self {
        Self {
            ref_time: Instant::now(),
            start_time: Utc::now(),
            streams: Vec::new(),
        }
    }
}

impl RecorderStream {
    pub fn complete(&mut self) -> RecordedStream {
        let n_channels = self.definition.n_channels();

        // Create a data buffer to dump (unsorted) measurements into
        let mut unsorted_data = Vec::new();

        // Drain the buffer, appending data to buffer, collecting timestamps and indices
        let mut samples = iter::from_fn(|| self.buffer.extract(&mut unsorted_data, n_channels))
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
        let definition = self.definition.clone();

        RecordedStream {
            data,
            data_timestamps_us,
            definition,
        }
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

impl StreamHandle {
    pub async fn add(&self, channels: &[f32]) {
        if self.recorder.recording.load(Ordering::SeqCst) {
            let mut lock = self.recorder.shared.lock().await;

            let time_us = lock.elapsed_micros();
            let rec_stream = &mut lock.streams[self.stream_id];

            // Check that the number of channels matches the stream definition
            debug_assert_eq!(channels.len(), rec_stream.definition.channels.len());

            rec_stream.buffer.append(time_us, channels);
        }
    }
}

/* == RecorderBuffer == */

struct RecorderBuffer(ChunkedBytes);

impl RecorderBuffer {
    pub fn new() -> Self {
        Self(ChunkedBytes::new())
    }

    pub fn append(&mut self, relative_time: u32, channels: &[f32]) {
        let buffer = &mut self.0;
        buffer.put_u32_ne(relative_time);
        channels.iter().for_each(|v| buffer.put_f32_ne(*v));
    }

    pub fn extract(&mut self, data: &mut Vec<f32>, n_channels: usize) -> Option<u32> {
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

    pub fn decode<R: Read>(reader: &mut R) -> io::Result<Self> {
        bincode::decode_from_std_read(reader, compact_config()).map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Failed to decode recording: {e}"),
            )
        })
    }

    pub fn encode<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        bincode::encode_into_std_write(self, writer, compact_config()).map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Failed to encode recording: {e}"),
            )
        })?;

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
