use std::sync::Arc;

use bytes::{Buf, BufMut};
use chunked_bytes::ChunkedBytes;
use tokio::{sync::Mutex, time::Instant};

#[derive(Default)]
pub struct Recorder {
    inner: Arc<Mutex<RecorderInner>>,
}

struct RecorderInner {
    buffer: RecordingBuffer,
    recording: bool,
    ref_time: Instant,
    streams: Vec<Stream>,
}

#[derive(Clone, Debug)]
pub struct Stream {
    channels: Box<[String]>,
    name: String,
}

pub struct StreamHandle {
    recorder: Arc<Mutex<RecorderInner>>,
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
        let mut inner = self.inner.lock().await;

        if inner.streams.len() >= u8::MAX as usize {
            panic!("Maximum number of streams reached");
        }

        let stream = Stream {
            name: name.to_string(),
            channels: channels
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .into_boxed_slice(),
        };

        let stream_id = inner.streams.len() as u8;
        inner.streams.push(stream.clone());

        let recorder = self.inner.clone();

        StreamHandle {
            recorder,
            stream_id,
        }
    }

    pub async fn start_recording(&self) {
        self.inner.lock().await.recording = true;
    }

    pub async fn stop_recording(&self) {
        self.inner.lock().await.recording = false;
    }

    pub async fn commit(&self) -> Recording {
        let mut lock = self.inner.lock().await;
        let streams = lock.streams.clone();
        lock.buffer.commit(streams)
    }

    pub async fn new_recording(&self) {
        self.inner.lock().await.buffer = RecordingBuffer::new();
    }

    pub async fn reset_reference_time(&self) {
        self.inner.lock().await.ref_time = Instant::now();
    }

    pub async fn streams(&self) -> Vec<Stream> {
        self.inner.lock().await.streams.clone()
    }
}

impl RecorderInner {
    fn elapsed_micros(&self) -> u32 {
        self.ref_time
            .elapsed()
            .as_micros()
            .try_into()
            .expect("Time overflow")
    }
}

impl Default for RecorderInner {
    fn default() -> Self {
        Self {
            buffer: RecordingBuffer::new(),
            recording: false,
            ref_time: Instant::now(),
            streams: Vec::new(),
        }
    }
}

impl StreamHandle {
    pub async fn add(&self, channels: &[f32]) {
        let mut lock = self.recorder.lock().await;

        debug_assert_eq!(
            channels.len(),
            lock.streams[self.stream_id as usize].channels.len()
        );

        if lock.recording {
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

    pub fn commit(&mut self, streams: Vec<Stream>) -> Recording {
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
        }
    }
}

/* == Recording == */

pub struct Recording {
    samples: Vec<Sample>,
    sample_data: Vec<f32>,
    streams: Vec<Stream>,
}

#[derive(Debug)]
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
    pub fn iter(&self) -> impl Iterator<Item = Measurement<'_>> + '_ {
        self.samples.iter().map(move |sample| {
            let data = &self.sample_data[sample.data_index..];
            let stream = &self.streams[sample.stream_id as usize];
            let date_index_end = sample.data_index + stream.channels.len();
            let data = &data[sample.data_index..date_index_end];

            Measurement { data, sample }
        })
    }

    pub fn sort(&mut self) {
        self.samples.sort_by_key(|s| s.time);
    }
}
