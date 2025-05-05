use std::{
    io::{Read, Write},
    iter,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};

use bytes::{Buf, BufMut};
use chrono::{DateTime, Utc};
use chunked_bytes::ChunkedBytes;
use eyre::{bail, eyre, Result};
use serde::{Deserialize, Serialize};
use tokio::{
    sync::{Mutex, MutexGuard},
    time::Instant,
};

use crate::{
    data::SegmentedRecordingIterator,
    misc::{
        buf::{BufExt, BufMutExt},
        plot_juggler::PlotJugglerBroadcaster,
    },
};

const MAGIC_NUMBER: u32 = u32::from_be_bytes(*b"RCDG");
const FORMAT_VERSION: u8 = 2;

#[derive(Default)]
pub struct Sink {
    inner: Arc<Inner>,
}

pub struct StreamWriter {
    definition: Arc<StreamDefinition>,
    inner: Arc<Inner>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
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
    markers: Vec<Marker>,
    plot_juggler: Option<PlotJugglerBroadcaster>,
    reference_instant: Instant,
    reference_time: DateTime<Utc>,
    streams: Vec<Stream>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Marker {
    duration: u32,
    name: String,
    time_us: u32,
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

    pub async fn add_stream_size<const N: usize>(
        &self,
        name: &str,
        channels: [&str; N],
    ) -> Result<StreamWriter> {
        self.add_stream(name.to_owned(), channels.map(str::to_owned).to_vec())
            .await
    }

    pub async fn remove_stream(&self, name: &str) -> bool {
        let mut lock = self.inner.shared.lock().await;
        let maybe_stream = lock.find_stream_with_name(name);

        if let Some(i) = maybe_stream {
            lock.streams.swap_remove(i);
        }

        maybe_stream.is_some()
    }

    pub async fn has_stream(&self, name: &str) -> bool {
        let lock = self.inner.shared.lock().await;
        lock.streams.iter().any(|s| s.definition.name == name)
    }

    /* == Recording == */

    pub async fn add_marker(&self, name: String) {
        self.add_marker_with_duration(name, 0).await
    }

    pub async fn add_marker_with_duration(&self, name: String, duration: u32) {
        let mut lock = self.inner.shared.lock().await;
        let time_us = lock.elapsed_micros();

        lock.markers.push(Marker {
            name,
            time_us,
            duration,
        });
    }

    pub async fn create_span(&self, marker: &str) -> Result<()> {
        let mut lock = self.inner.shared.lock().await;

        let time_us = lock.elapsed_micros();

        let first_marker = lock
            .markers
            .iter_mut()
            .rev()
            .find(|m| m.name == marker)
            .ok_or_else(|| eyre!("No span found for marker {}", marker))?;

        if first_marker.duration == 0 {
            first_marker.duration = time_us - first_marker.time_us;
        }

        Ok(())
    }

    pub fn set_record(&self, active: bool) {
        self.inner.recording.store(active, Ordering::SeqCst);
    }

    pub async fn complete(&self) -> Recording {
        self.set_record(false);

        let mut lock = self.inner.shared.lock().await;
        tracing::debug!("Completed recording");

        let markers = lock.markers.clone();
        let start_timestamp_us = lock.reference_time.timestamp_micros();
        let recorded_streams = lock.streams.iter_mut().map(Stream::complete).collect();

        Recording {
            markers,
            recorded_streams,
            start_timestamp_us,
        }
    }

    pub async fn clear_buffer(&self) {
        let mut lock = self.inner.shared.lock().await;

        lock.markers.clear();

        for stream in &mut lock.streams {
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
            markers: Vec::new(),
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

#[derive(Clone, Deserialize, Serialize)]
pub struct Recording {
    pub markers: Vec<Marker>,
    pub start_timestamp_us: i64,
    pub recorded_streams: Vec<RecordedStream>,
}

#[derive(Clone, Deserialize, Serialize)]
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
    pub fn segment(&self, divisions: u32) -> Option<SegmentedRecordingIterator<'_>> {
        SegmentedRecordingIterator::new(self, divisions)
    }

    /* == Encoding & decoding == */

    pub fn decode<B: Buf>(buf: &mut B) -> Result<Self> {
        RecordingDecoder::new()
            .decode(buf)
            .ok_or_else(|| eyre!("Failed to decode recording"))
    }

    pub fn decode_reader<R: Read>(reader: &mut R) -> Result<Self> {
        let mut buf = Vec::new();
        reader.read_to_end(&mut buf)?;
        Self::decode(&mut &buf[..])
    }

    pub fn encode_writer<W: Write>(&self, writer: &mut W) -> Result<()> {
        let mut buf = Vec::new();
        self.encode(&mut buf);
        writer.write_all(&buf)?;
        Ok(())
    }

    pub fn encode<B: BufMut>(&self, buf: &mut B) {
        RecordingEncoder(self).encode(buf);
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

/* == Encoding/Decoding == */

struct RecordingEncoder<'a>(&'a Recording);

impl RecordingEncoder<'_> {
    pub fn encode<B: BufMut>(&self, buf: &mut B) {
        buf.put_u32(MAGIC_NUMBER);
        buf.put_u8(FORMAT_VERSION);
        buf.put_i64(self.0.start_timestamp_us);

        self.encode_markers(buf);
        self.encode_definitions(buf);
        self.encode_timestamps(buf);
        self.encode_data(buf);
    }

    fn encode_markers<B: BufMut>(&self, buf: &mut B) {
        buf.put_u8(self.0.markers.len() as u8);

        for marker in &self.0.markers {
            buf.put_u32(marker.time_us);
            buf.put_u32(marker.duration);
            buf.put_string(&marker.name);
        }
    }

    fn encode_definitions<B: BufMut>(&self, buf: &mut B) {
        buf.put_u8(self.0.recorded_streams.len() as u8);

        for RecordedStream { definition, .. } in &self.0.recorded_streams {
            buf.put_string(&definition.name);
            buf.put_u8(definition.n_channels() as u8);

            for channel in &definition.channels {
                buf.put_string(channel);
            }
        }
    }

    fn encode_timestamps<B: BufMut>(&self, buf: &mut B) {
        for RecordedStream {
            data_timestamps_us: timestamps,
            ..
        } in &self.0.recorded_streams
        {
            buf.put_u32(timestamps.len() as u32);

            for timestamp in timestamps {
                buf.put_u32(*timestamp);
            }
        }
    }

    fn encode_data<B: BufMut>(&self, buf: &mut B) {
        for stream in &self.0.recorded_streams {
            for datum in &stream.data {
                buf.put_f32(*datum);
            }
        }
    }
}

#[derive(Default)]
struct RecordingDecoder {
    definitions: Vec<StreamDefinition>,
    markers: Vec<Marker>,
    timestamps: Vec<Vec<u32>>,
    data: Vec<Vec<f32>>,
    start_timestamp_us: i64,
}

impl RecordingDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    fn decode<B: Buf>(mut self, buf: &mut B) -> Option<Recording> {
        if buf.try_get_u32().ok()? != MAGIC_NUMBER {
            return None;
        }

        if buf.try_get_u8().ok()? != FORMAT_VERSION {
            return None;
        }

        self.start_timestamp_us = buf.try_get_i64().ok()?;

        self.decode_markers(buf)?;
        self.decode_definitions(buf)?;
        self.decode_timestamps(buf)?;
        self.decode_data(buf)?;

        Some(self.finalise())
    }

    fn decode_markers<B: Buf>(&mut self, buf: &mut B) -> Option<()> {
        for _ in 0..buf.try_get_u8().ok()? {
            self.markers.push(Marker {
                time_us: buf.try_get_u32().ok()?,
                duration: buf.try_get_u32().ok()?,
                name: buf.try_get_string()?,
            });
        }

        Some(())
    }

    fn decode_definitions<B: Buf>(&mut self, buf: &mut B) -> Option<()> {
        for _ in 0..buf.try_get_u8().ok()? {
            let name = buf.try_get_string()?;

            let channels = (0..buf.try_get_u8().ok()?)
                .map(|_| buf.try_get_string())
                .collect::<Option<_>>()?;

            self.definitions.push(StreamDefinition { name, channels });
        }

        Some(())
    }

    fn decode_timestamps<B: Buf>(&mut self, buf: &mut B) -> Option<()> {
        for _ in 0..self.definitions.len() {
            let size = buf.try_get_u32().ok()? as usize;
            let mut stream_timestamps = Vec::with_capacity(size);

            for _ in 0..size {
                stream_timestamps.push(buf.try_get_u32().ok()?);
            }

            self.timestamps.push(stream_timestamps);
        }

        Some(())
    }

    fn decode_data<B: Buf>(&mut self, buf: &mut B) -> Option<()> {
        for (definition, timestamps) in self.definitions.iter().zip(self.timestamps.iter()) {
            let size = timestamps.len() as usize * definition.n_channels();
            let mut stream_data = Vec::with_capacity(size);

            for _ in 0..size {
                stream_data.push(buf.try_get_f32().ok()?);
            }

            self.data.push(stream_data);
        }

        Some(())
    }

    fn finalise(self) -> Recording {
        let mut recorded_streams = Vec::with_capacity(self.definitions.len());

        self.definitions
            .into_iter()
            .zip(self.data.into_iter())
            .zip(self.timestamps.into_iter())
            .for_each(|((definition, data), data_timestamps_us)| {
                recorded_streams.push(RecordedStream {
                    definition,
                    data_timestamps_us,
                    data,
                });
            });

        let markers = self.markers;
        let start_timestamp_us = self.start_timestamp_us;

        Recording {
            markers,
            start_timestamp_us,
            recorded_streams,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_recording() {
        let recording = Recording {
            markers: vec![
                Marker {
                    name: "TestMarker".to_string(),
                    time_us: 123456,
                    duration: 0,
                },
                Marker {
                    name: "AnotherMarker".to_string(),
                    time_us: 654321,
                    duration: 9876,
                },
            ],
            start_timestamp_us: 123456789,
            recorded_streams: vec![
                RecordedStream {
                    definition: StreamDefinition {
                        name: "TestStream".to_string(),
                        channels: vec!["Channel1".to_string(), "Channel2".to_string()],
                    },
                    data_timestamps_us: vec![1, 2, 3],
                    data: vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                },
                RecordedStream {
                    definition: StreamDefinition {
                        name: "AnotherStream".to_string(),
                        channels: vec!["ChannelA".to_string(), "ChannelB".to_string()],
                    },
                    data_timestamps_us: vec![4, 5, 6],
                    data: vec![7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
                },
            ],
        };

        let mut buf = Vec::new();
        recording.encode(&mut buf);

        let decoded_recording = Recording::decode(&mut &buf[..]).unwrap();

        // Compare the JSON representation for equality (key ordering is stable)
        assert_eq!(
            serde_json::to_string(&recording).unwrap(),
            serde_json::to_string(&decoded_recording).unwrap()
        );
    }
}
