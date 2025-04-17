use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

use bincode::{Decode, Encode};
use bytes::{Buf, BufMut};
use chrono::{DateTime, Utc};
use chunked_bytes::ChunkedBytes;
use tokio::{sync::Mutex, time::Instant};

use crate::data::SegmentedRecordingIterator;

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
    buffer: RecordingBuffer,
    ref_time: Instant,
    start_time: DateTime<Utc>,
    streams: Vec<Stream>,
}

#[derive(Clone, Debug, Decode, Encode)]
pub struct Stream {
    pub name: String,
    pub channels: Vec<String>,
}

pub struct StreamHandle {
    recorder: Arc<RecorderInner>,
    stream_id: u8,
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

        let name = name.to_string();
        tracing::info!("Added stream: {name}");

        let stream = Stream {
            name,
            channels: channels.iter().map(ToString::to_string).collect(),
        };

        let stream_id = lock.streams.len() as u8;
        lock.streams.push(stream.clone());

        let recorder = self.inner.clone();

        StreamHandle {
            recorder,
            stream_id,
        }
    }

    pub fn start_recording(&self) {
        tracing::debug!("Recording resumed");
        self.inner.recording.store(true, Ordering::SeqCst);
    }

    pub fn stop_recording(&self) {
        tracing::debug!("Recording paused");
        self.inner.recording.store(false, Ordering::SeqCst);
    }

    pub async fn finalise(&self) -> Recording {
        self.stop_recording();
        let mut lock = self.inner.shared.lock().await;

        let streams = lock.streams.clone();
        let start_time = lock.start_time;
        let recording = lock.buffer.commit(&start_time, streams);

        tracing::info!("Recording finalised");
        recording
    }

    pub async fn clear_buffer(&self) {
        tracing::info!("Recording buffer cleared");
        self.inner.shared.lock().await.buffer = RecordingBuffer::new();
    }

    pub async fn reset_reference_time(&self) {
        tracing::info!("Recording reference time reset");
        self.inner.shared.lock().await.ref_time = Instant::now();
        self.inner.shared.lock().await.start_time = Utc::now();
    }

    pub async fn streams(&self) -> Vec<Stream> {
        self.inner.shared.lock().await.streams.clone()
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
            buffer: Default::default(),
            ref_time: Instant::now(),
            start_time: Utc::now(),
            streams: Default::default(),
        }
    }
}

impl StreamHandle {
    pub async fn add(&self, channels: &[f32]) {
        if self.recorder.recording.load(Ordering::SeqCst) {
            let mut lock = self.recorder.shared.lock().await;

            // Check that the number of channels matches the stream config
            debug_assert_eq!(
                channels.len(),
                lock.streams[self.stream_id as usize].channels.len()
            );

            let time = lock.elapsed_micros();
            lock.buffer.add(time, self.stream_id, channels);
        }
    }
}

/* == RecordingBuffer == */

#[derive(Default)]
struct RecordingBuffer {
    buffer: ChunkedBytes,
}

impl RecordingBuffer {
    fn new() -> Self {
        Self::default()
    }

    fn add(&mut self, time: u32, stream_id: u8, channels: &[f32]) {
        self.buffer.put_u32(time);
        self.buffer.put_u8(stream_id);
        self.buffer.put_u8(channels.len() as u8);

        for &value in channels {
            self.buffer.put_f32(value);
        }
    }

    pub fn commit(&mut self, time: &DateTime<Utc>, streams: Vec<Stream>) -> Recording {
        let mut samples = Vec::new();
        let mut sample_data = Vec::new();

        loop {
            if self.buffer.is_empty() {
                break;
            }

            let time = self.buffer.get_u32();
            let stream_id = self.buffer.get_u8();
            let n_channels = self.buffer.get_u8();

            samples.push(Sample {
                time,
                stream_id,
                data_index: sample_data.len(),
            });

            sample_data.extend((0..n_channels).map(|_| self.buffer.get_f32()));
        }

        debug_assert!(self.buffer.is_empty());

        samples.shrink_to_fit();
        sample_data.shrink_to_fit();

        Recording {
            samples,
            sample_data,
            streams,
            time: time.to_rfc3339(),
        }
    }
}

/* == Recording == */

#[derive(Decode, Encode)]
pub struct Recording {
    pub streams: Vec<Stream>,
    pub samples: Vec<Sample>,
    pub sample_data: Vec<f32>,
    pub time: String,
}

#[derive(Debug, Decode, Encode)]
pub struct Sample {
    pub time: u32,
    pub stream_id: u8,
    pub data_index: usize,
}

#[derive(Debug)]
pub struct Measurement<'a> {
    pub data: &'a [f32],
    pub sample: &'a Sample,
}

impl Recording {
    pub fn get_at(&self, index: usize) -> Measurement<'_> {
        self.measurement(&self.samples[index])
    }

    pub fn iter(&self) -> impl ExactSizeIterator<Item = Measurement<'_>> + '_ {
        self.samples.iter().map(move |s| Self::measurement(self, s))
    }

    pub fn iter_stream(&self, stream_id: u8) -> impl Iterator<Item = Measurement<'_>> + '_ {
        self.iter().filter(move |s| s.sample.stream_id == stream_id)
    }

    pub fn n_channels(&self) -> usize {
        self.streams.iter().map(|s| s.channels.len()).sum()
    }

    pub fn segment(&self, divisions: u32) -> Option<SegmentedRecordingIterator<'_>> {
        SegmentedRecordingIterator::new(self, divisions)
    }

    fn measurement<'a>(&'a self, sample: &'a Sample) -> Measurement<'a> {
        let stream = &self.streams[sample.stream_id as usize];
        let date_index_end = sample.data_index + stream.channels.len();
        let data = &self.sample_data[sample.data_index..date_index_end];

        Measurement { data, sample }
    }

    pub fn sort(&mut self) {
        self.samples.sort_by_key(|s| s.time);
    }
}
