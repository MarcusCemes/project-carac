use tokio::io::{self, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

use crate::{
    data::time::SegmentedRecordingIterator,
    misc::buf::{ReadExt, WriteExt},
};

/* === Definitions === */

pub struct Run {
    pub recorded_streams: Vec<RecordedStream>,
}

#[derive(Clone, Debug)]
pub struct StreamInfo {
    pub name: String,
    pub channels: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct ChannelInfo {
    pub name: String,
    pub unit: String,
}

pub struct RecordedStream {
    pub channel_data: Vec<f32>,
    pub timestamps: Vec<u32>,
}

pub struct RunSample<'a> {
    pub channel_data: &'a [f32],
    pub delta_us: u32,
}

pub struct RunView<'a> {
    recording: &'a Run,
}

/* === Implementations === */

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

    pub async fn read<R: AsyncRead + Unpin>(r: &mut R, streams: &[StreamInfo]) -> io::Result<Self> {
        let mut recorded_streams = Vec::new();

        for stream in streams {
            let n_timestamps = r.read_u32().await? as usize;
            let n_samples = n_timestamps * stream.channels.len();

            let mut timestamps = Vec::with_capacity(n_timestamps);
            let mut channel_data = Vec::with_capacity(n_samples);

            for _ in 0..n_timestamps {
                timestamps.push(r.read_u32().await?);
            }

            for _ in 0..n_samples {
                channel_data.push(r.read_f32().await?);
            }

            recorded_streams.push(RecordedStream::new(timestamps, channel_data));
        }

        Ok(Self { recorded_streams })
    }

    pub async fn write<W: AsyncWrite + Unpin>(&self, w: &mut W) -> io::Result<()> {
        for stream in &self.recorded_streams {
            w.write_u32(stream.timestamps.len() as u32).await?;

            for timestamp in &stream.timestamps {
                w.write_u32(*timestamp).await?;
            }

            for data in &stream.channel_data {
                w.write_f32(*data).await?;
            }
        }

        Ok(())
    }
}

impl StreamInfo {
    pub fn qualified_channel_names(&self) -> Vec<String> {
        self.channels
            .iter()
            .map(|channel| format!("{}/{}", self.name, channel))
            .collect()
    }

    pub async fn read<R: AsyncRead + Unpin>(r: &mut R) -> io::Result<Self> {
        let name = r.read_string().await?;
        let n_channels = r.read_u8().await? as usize;

        let mut channels = Vec::with_capacity(n_channels);

        for _ in 0..n_channels {
            channels.push(r.read_string().await?);
        }

        Ok(Self { name, channels })
    }

    pub async fn write<W: AsyncWrite + Unpin>(&self, w: &mut W) -> io::Result<()> {
        w.write_string(&self.name).await?;
        w.write_u8(self.channels.len() as u8).await?;

        for channel in &self.channels {
            w.write_string(channel).await?;
        }

        Ok(())
    }
}

impl ChannelInfo {
    pub fn new(name: String, unit: Option<String>) -> Self {
        let unit = unit.unwrap_or_else(|| "-".to_string());
        Self { name, unit }
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

    pub fn n_channels(&self) -> usize {
        self.channel_data.len() / self.timestamps.len().max(1) // panics if zero
    }
}

impl RunSample<'_> {
    pub fn time_s(&self) -> f32 {
        self.delta_us as f32 * 1e-6
    }
}

// impl RunView<'_> {
//     pub fn segment(&self, divisions: u32, streams: &[StreamInfo]) {
//         todo!()
//     }
// }
