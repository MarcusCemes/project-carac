use std::{iter, path::Path};

use bincode::{Decode, Encode};
use bytes::{Buf, BufMut};
use chunked_bytes::ChunkedBytes;
use eyre::{Context, ContextCompat, Result, bail};
use polars::prelude::*;
use tokio::{fs::File, io::AsyncReadExt};

use crate::{
    data::{sink::StreamInfo, time::SegmentedRecordingIterator},
    misc::standard_config,
};

use super::session::SessionMetadata;

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

#[derive(Clone, Debug, Decode, Encode)]
pub struct RecordedStream {
    pub channel_data: Vec<f32>,
    pub timestamps: Vec<u32>,
}

pub struct RunSample<'a> {
    pub channel_data: &'a [f32],
    pub delta_us: u32,
}

/* === Implementations === */

impl Experiment {
    pub async fn load(path: &Path) -> Result<Self> {
        let mut file = File::open(path).await?;
        let mut buf = ChunkedBytes::new();

        file.read_buf(&mut buf).await?;

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

        let runs = iter::from_fn(|| {
            buf.has_remaining()
                .then(|| Run::decode(buf, &header.streams))
        })
        .collect::<Result<Vec<_>>>()?;

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

    pub fn segment<'a>(
        &'a self,
        streams: &'a [StreamInfo],
        divisions: u32,
    ) -> Option<SegmentedRecordingIterator<'a>> {
        SegmentedRecordingIterator::new(self, streams, divisions)
    }

    pub fn dataframe(&self, streams: &[StreamInfo], divisions: u32) -> Result<DataFrame> {
        let n_channels = streams.iter().map(|s| s.channels.len()).sum();

        let mut data_buffer = vec![0.; n_channels];
        let mut time_series = Vec::with_capacity(divisions as usize);

        let mut series = iter::repeat_with(|| Vec::with_capacity(divisions as usize))
            .take(n_channels)
            .collect::<Vec<_>>();

        let mut iterator = self
            .segment(streams, divisions)
            .wrap_err("Segmentation failed")?;

        while let Some(time) = iterator.next(&mut data_buffer) {
            for (data, series) in data_buffer.iter().zip(series.iter_mut()) {
                series.push(*data);
            }

            time_series.push(time);
        }

        let time_column =
            ChunkedArray::<Float32Type>::from_vec("time".into(), time_series).into_column();

        let channel_columns = series
            .into_iter()
            .zip(streams.iter().flat_map(|s| s.qualified_channel_names()))
            .map(|(series, name)| {
                ChunkedArray::<Float32Type>::from_vec(name.into(), series).into_column()
            });

        let columns = iter::once(time_column).chain(channel_columns).collect();

        DataFrame::new(columns).wrap_err("Failed to create DataFrame")
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
                timestamps.push(buf.try_get_u32()?);

                for _ in 0..n_channels {
                    data.push(buf.try_get_f32()?);
                }
            }

            recorded_streams.push(RecordedStream::new(timestamps, data));
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
                .zip(stream.channel_data.chunks_exact(stream.n_channels()))
            {
                buf.put_u32(*timestamp);

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
    pub fn new(timestamps: Vec<u32>, channel_data: Vec<f32>) -> Self {
        Self {
            timestamps,
            channel_data,
        }
    }

    pub fn iter_samples(&self) -> impl Iterator<Item = RunSample> {
        self.timestamps
            .iter()
            .zip(self.channel_data.chunks_exact(self.n_channels()))
            .map(|(&delta_us, channel_data)| RunSample {
                channel_data,
                delta_us,
            })
    }

    /// Estimates the number of channels based on contained data. If there is
    /// no data, assumes a single channel.
    pub fn n_channels(&self) -> usize {
        self.channel_data.len().max(1) / self.timestamps.len().max(1)
    }
}

impl RunSample<'_> {
    pub fn time_s(&self) -> f32 {
        self.delta_us as f32 * 1e-6
    }
}
