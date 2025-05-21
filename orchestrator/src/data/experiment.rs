use std::{
    path::Path,
    slice::{ChunksExact, Iter},
};

use bincode::{Decode, Encode};
use bytes::{Buf, BufMut};
use chunked_bytes::ChunkedBytes;
use eyre::{Context, ContextCompat, Result, bail};
use polars::prelude::*;
use tokio::fs;

use crate::{data::sink::StreamInfo, misc::standard_config};

use super::{
    processing::{SegmentationMethod, SegmentedRun},
    session::SessionMetadata,
};

/* === Definitions === */

const MAGIC_NUMBER: [u8; 4] = *b"EXPT";
const FORMAT_VERSION: u8 = 1;

#[derive(Clone, Debug)]
pub struct Experiment {
    pub header: ExperimentHeader,
    pub runs: Vec<Run>,
}

#[derive(Clone, Debug, Decode, Encode)]
pub struct ExperimentHeader {
    magic_number: [u8; 4],
    version: u8,
    pub name: String,
    pub streams: Box<[u8]>,
}

#[derive(Clone, Debug)]
pub struct Run {
    pub recorded_streams: Vec<RecordedStream>,
}

#[derive(Clone, Debug)]
pub struct RecordedStream {
    pub channel_data: Vec<f32>,
    pub n_channels: usize,
    pub timestamps: Vec<SampleTime>,
}

pub struct RunSampleIterator<'a> {
    iter_timestamp: Iter<'a, SampleTime>,
    iter_data: ChunksExact<'a, f32>,
}

pub struct RunSample<'a> {
    pub channel_data: &'a [f32],
    pub time: SampleTime,
}

pub struct ChannelData(pub Box<[f32]>);

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct SampleTime(pub u32);

/* === Implementations === */

impl Experiment {
    pub async fn load(path: &Path) -> Result<Self> {
        let contents = fs::read(path).await?;
        let mut buf = &contents[..];

        Self::decode(&mut buf).wrap_err("Failed to decode experiment")
    }

    pub fn stream_names(&self, streams: Option<&[StreamInfo]>) -> impl Iterator<Item = String> {
        let streams = streams.unwrap_or(&[]);

        (0..self.header.streams.len()).map(move |i| {
            streams
                .get(i)
                .map_or_else(|| format!("stream_{}", i), |s| s.name.clone())
        })
    }

    pub fn guess_metadata(&self) -> SessionMetadata {
        let streams = self
            .header
            .streams
            .iter()
            .enumerate()
            .map(|(i, n)| StreamInfo {
                name: format!("stream_{i:2}"),
                channels: vec![format!("channel_{n:2}")],
            })
            .collect();

        SessionMetadata { streams }
    }

    pub fn decode<B: Buf>(buf: &mut B) -> Result<Self> {
        let header: ExperimentHeader =
            bincode::decode_from_std_read(&mut buf.reader(), standard_config())?;

        if header.magic_number != MAGIC_NUMBER {
            bail!("Invalid magic number");
        }

        if header.version != FORMAT_VERSION {
            bail!("Unsupported format version");
        }

        let mut runs = Vec::new();

        while buf.has_remaining() {
            runs.push(Run::decode(buf, &header.streams)?);
        }

        Ok(Experiment { header, runs })
    }

    pub fn encode<B: BufMut>(&self, buf: &mut B) -> Result<()> {
        bincode::encode_into_std_write(&self.header, &mut buf.writer(), standard_config())?;

        for run in &self.runs {
            run.encode(buf);
        }

        Ok(())
    }
}

impl ExperimentHeader {
    pub fn new(name: String, streams: Box<[u8]>) -> Self {
        Self {
            magic_number: MAGIC_NUMBER,
            version: FORMAT_VERSION,
            name,
            streams,
        }
    }
}

impl Run {
    pub fn new(recorded_streams: Vec<RecordedStream>) -> Self {
        Self { recorded_streams }
    }

    pub fn get_stream(&self, name: &str, streams: &[StreamInfo]) -> Option<&RecordedStream> {
        streams
            .iter()
            .zip(self.recorded_streams.iter())
            .find_map(|(info, stream)| (info.name == name).then_some(stream))
    }

    pub fn get_stream_mut(
        &mut self,
        name: &str,
        streams: &[StreamInfo],
    ) -> Option<&mut RecordedStream> {
        streams
            .iter()
            .zip(self.recorded_streams.iter_mut())
            .find_map(|(info, stream)| (info.name == name).then_some(stream))
    }

    pub fn duration(&self) -> Option<SampleTime> {
        self.recorded_streams
            .iter()
            .flat_map(|s| s.timestamps.last())
            .copied()
            .max()
    }

    pub fn total_channels(&self) -> usize {
        self.recorded_streams.iter().map(|s| s.n_channels).sum()
    }

    pub fn dataframe(&self, streams: &[StreamInfo], divisions: u32) -> Result<DataFrame> {
        SegmentedRun::new(SegmentationMethod::Count(divisions))
            .columns(self)
            .wrap_err("Segmentation failed")?
            .dataframe(streams)
    }

    pub fn decode<B: Buf>(buf: &mut B, channels: &[u8]) -> Result<Run> {
        let block_size = buf.try_get_u32()? as usize;
        let mut buf = buf.take(block_size);

        let mut recorded_streams = Vec::with_capacity(channels.len());

        for &n_channels in channels {
            let n_timestamps = buf.try_get_u32()? as usize;

            let mut timestamps = Vec::with_capacity(n_timestamps);
            let mut data = Vec::with_capacity(n_channels as usize * n_timestamps);

            for _ in 0..n_timestamps {
                timestamps.push(SampleTime(buf.try_get_u32()?));

                for _ in 0..n_channels {
                    data.push(buf.try_get_f32()?);
                }
            }

            recorded_streams.push(RecordedStream::new(timestamps, data, n_channels as usize));
        }

        Ok(Run { recorded_streams })
    }

    pub fn encode<B: BufMut>(&self, buf: &mut B) {
        let old_buf = buf;

        let mut buf = ChunkedBytes::new();

        for stream in &self.recorded_streams {
            buf.put_u32(stream.timestamps.len() as u32);

            for (timestamp, data) in stream
                .timestamps
                .iter()
                .zip(stream.channel_data.chunks_exact(stream.n_channels))
            {
                buf.put_u32(timestamp.0);

                for datum in data {
                    buf.put_f32(*datum);
                }
            }
        }

        old_buf.put_u32(buf.remaining() as u32);
        old_buf.put(buf);
    }
}

impl RecordedStream {
    pub fn new(timestamps: Vec<SampleTime>, channel_data: Vec<f32>, n_channels: usize) -> Self {
        Self {
            channel_data,
            timestamps,
            n_channels,
        }
    }

    pub fn time_column(&self) -> Vec<f32> {
        self.timestamps.iter().copied().map(f32::from).collect()
    }

    pub fn columns(&self) -> Vec<Box<[f32]>> {
        let n_samples = self.timestamps.len();
        let n_channels = self.n_channels;

        assert_eq!(self.channel_data.len(), n_samples * n_channels);

        let mut columns = (0..n_channels)
            .map(|_| {
                let buffer = Box::new_uninit_slice(n_samples);

                // SAFETY: the buffer is completely filled below
                unsafe { buffer.assume_init() }
            })
            .collect::<Vec<_>>();

        for i in 0..n_samples {
            for j in 0..n_channels {
                // SAFETY: the length of channel_data is asserted above
                unsafe {
                    let source = self.channel_data.get_unchecked(i * n_channels + j);
                    let destination = columns.get_unchecked_mut(j).get_unchecked_mut(i);

                    *destination = *source;
                }
            }
        }

        columns
    }

    pub fn iter_samples(&self) -> RunSampleIterator<'_> {
        let iter_timestamp = self.timestamps.iter();
        let iter_data = self.channel_data.chunks_exact(self.n_channels);

        RunSampleIterator {
            iter_timestamp,
            iter_data,
        }
    }
}

impl<'a> Iterator for RunSampleIterator<'a> {
    type Item = RunSample<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let maybe_timestamp = self.iter_timestamp.next();
        let maybe_data = self.iter_data.next();

        maybe_timestamp
            .zip(maybe_data)
            .map(|(&time, channel_data)| RunSample { channel_data, time })
    }
}

impl From<SampleTime> for f32 {
    fn from(value: SampleTime) -> Self {
        1e-6 * value.0 as f32
    }
}
