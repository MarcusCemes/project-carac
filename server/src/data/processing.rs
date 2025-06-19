use std::{
    io::Write,
    iter::{self, Peekable},
    sync::Arc,
};

use butterworth::{Cutoff, Filter};
use eyre::{ContextCompat, Result, eyre};
use nalgebra::{Rotation3, Vector3};
use parquet::{
    basic::{Repetition, Type as PhysicalType},
    data_type::FloatType,
    errors::ParquetError,
    file::{properties::WriterProperties, writer::SerializedFileWriter},
    schema::types::Type,
};

use crate::defs::{Load, Point};

use super::{
    experiment::{RecordedStream, Run, RunSample, RunSampleIterator},
    sink::StreamInfo,
};

#[derive(Debug, Clone, Copy)]
pub struct SegmentationOptions {
    _method: SegmentationMethod,
}

#[derive(Debug, Clone, Copy)]
pub enum SegmentationMethod {
    Count(u32),
    Frequency(f32),
}

pub struct SegmentedRun {
    method: SegmentationMethod,
}

pub struct SegmentedRunRows<'a> {
    interpolators: Box<[StreamInterpolator<'a>]>,
    time: TimeIterator,
}

pub struct SegmentedRunColumns {
    pub time: Vec<f32>,
    pub streams: Vec<Vec<f32>>,
}

impl SegmentedRun {
    pub fn new(method: SegmentationMethod) -> Self {
        Self { method }
    }

    pub fn write_csv<W: Write>(&self, buf: W, run: &Run, streams: &[StreamInfo]) -> Result<()> {
        let (mut rows, mut values) = self.rows(run)?;

        let mut w = csv::Writer::from_writer(buf);

        w.write_record(column_names(streams))?;

        while let Some(time) = rows.next(&mut values) {
            w.write_field(format!("{time}"))?;

            for value in &values {
                w.write_field(format!("{value}"))?;
            }

            w.write_record(None::<&[u8]>)?;
        }

        Ok(())
    }

    pub fn write_parquet<W: Write + Send>(
        &self,
        buf: W,
        run: &Run,
        streams: &[StreamInfo],
    ) -> Result<()> {
        let columns = self.columns(run)?;
        let schema = Self::parquet_schema(streams)?;

        let props = Arc::new(WriterProperties::builder().build());

        let mut writer = SerializedFileWriter::new(buf, schema, props)?;
        let mut row_group_writer = writer.next_row_group()?;

        let column_data = iter::once(&columns.time).chain(&columns.streams);

        for column in column_data {
            let mut col_writer = row_group_writer.next_column()?.unwrap();

            col_writer
                .typed::<FloatType>()
                .write_batch(column, None, None)?;

            col_writer.close()?;
        }

        row_group_writer.close()?;
        writer.close()?;

        Ok(())
    }

    fn parquet_schema(streams: &[StreamInfo]) -> Result<Arc<Type>> {
        let column_names = iter::once("time".to_owned())
            .chain(streams.iter().flat_map(StreamInfo::qualified_channel_names));

        let fields = column_names
            .map(|name| {
                Type::primitive_type_builder(&name, PhysicalType::FLOAT)
                    .with_repetition(Repetition::REQUIRED)
                    .build()
                    .map(Arc::new)
            })
            .collect::<Result<Vec<_>, ParquetError>>()?;

        let schema = Type::group_type_builder("schema")
            .with_fields(fields)
            .build()?;

        Ok(Arc::new(schema))
    }

    fn rows<'a>(&self, run: &'a Run) -> Result<(SegmentedRunRows<'a>, Box<[f32]>)> {
        let time = self.time_iterator(run)?;

        let n_channels = run.total_channels();
        let buffer = vec![0.0; n_channels].into_boxed_slice();

        Ok((SegmentedRunRows::new(run, time), buffer))
    }

    fn columns(&self, run: &Run) -> Result<SegmentedRunColumns> {
        let (mut rows, mut buffer) = self.rows(run)?;

        let n_samples = rows.time.total;
        let width = buffer.len();

        let mut time = Vec::with_capacity(n_samples);

        let mut streams = (0..width)
            .map(|_| Vec::with_capacity(n_samples))
            .collect::<Vec<_>>();

        while let Some(data) = rows.next(&mut buffer) {
            time.push(data);

            for i in 0..width {
                // SAFETY: streams and buffer are both of size (0..width)
                unsafe { streams.get_unchecked_mut(i).push(*buffer.get_unchecked(i)) };
            }
        }

        Ok(SegmentedRunColumns { time, streams })
    }

    fn time_iterator(&self, run: &Run) -> Result<TimeIterator> {
        let run_duration = run.duration().ok_or(eyre!("Run has no duration"))?;
        let run_duration = f32::from(run_duration);

        let (duration, count) = match self.method {
            SegmentationMethod::Count(count) => (run_duration, count),
            SegmentationMethod::Frequency(freq) => {
                let count = (run_duration * freq).floor() as u32;
                let duration = count as f32 / freq;

                (duration, count)
            }
        };

        Ok(TimeIterator::new(duration, count))
    }
}

impl<'a> SegmentedRunRows<'a> {
    pub fn new(run: &'a Run, time: TimeIterator) -> SegmentedRunRows<'a> {
        let interpolators = run
            .recorded_streams
            .iter()
            .map(StreamInterpolator::new)
            .collect::<Vec<_>>()
            .into_boxed_slice();

        SegmentedRunRows {
            interpolators,
            time,
        }
    }

    pub fn next(&mut self, data: &mut [f32]) -> Option<f32> {
        let time = self.time.next()?;

        let mut buf = data;

        for interpolator in &mut self.interpolators {
            let slice = &mut buf[..interpolator.channels];

            interpolator.next(time, slice);

            buf = &mut buf[interpolator.channels..];
        }

        assert_eq!(buf.len(), 0);

        Some(time)
    }
}

struct StreamInterpolator<'a> {
    channels: usize,
    cursor: Option<RunSample<'a>>,
    iter_samples: Peekable<RunSampleIterator<'a>>,
}

impl StreamInterpolator<'_> {
    fn new(recorded_stream: &RecordedStream) -> StreamInterpolator<'_> {
        let mut iter_samples = recorded_stream.iter_samples().peekable();
        let cursor = iter_samples.next();

        StreamInterpolator {
            channels: recorded_stream.n_channels,
            cursor,
            iter_samples,
        }
    }

    fn next(&mut self, time: f32, data: &mut [f32]) -> bool {
        let Some(cursor) = self.cursor.as_mut() else {
            return false;
        };

        while let Some(next_sample) = self.iter_samples.peek() {
            if f32::from(next_sample.time) > time {
                break;
            }

            // SAFETY: We just peeked the next sample, so it exists.
            *cursor = unsafe { self.iter_samples.next().unwrap_unchecked() };
        }

        // Now that the cursor is in the correct position, read its time
        let cursor_time = f32::from(cursor.time);

        let Some(next_cursor) = self.iter_samples.peek() else {
            data.copy_from_slice(cursor.channel_data);
            return true;
        };

        // Interpolate between the two samples
        let lt = cursor_time;
        let rt = f32::from(next_cursor.time);

        // Handle cases where samples have the same timestamp to avoid division by zero
        if lt == rt {
            data.copy_from_slice(cursor.channel_data);
            return true;
        }

        let t = (time - lt) / (rt - lt);

        // Don't extrapolate beyond the first or last sample
        let t = t.clamp(0., 1.);

        // Compute the channel interpolation and fill the output buffer
        let l = cursor.channel_data.iter();
        let r = next_cursor.channel_data.iter();
        let o = data.iter_mut();

        for ((&l, &r), o) in l.zip(r).zip(o) {
            *o = l + (r - l) * t;
        }

        true
    }
}

pub fn column_names(streams: &[StreamInfo]) -> Vec<String> {
    iter::once("time".to_owned())
        .chain(streams.iter().flat_map(|s| s.qualified_channel_names()))
        .collect()
}

/* == TimeIterator == */

pub struct TimeIterator {
    duration: f32,
    index: usize,
    total: usize,
}

impl TimeIterator {
    pub fn new(duration: f32, total: u32) -> Self {
        Self {
            duration,
            index: 0,
            total: total as usize,
        }
    }
}

impl Iterator for TimeIterator {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.total {
            return None;
        }

        let time = self.duration * (self.index as f32 / self.total as f32);
        self.index += 1;
        Some(time)
    }
}

/* == StreamFilter == */

pub struct StreamFilter {
    cutoff_frequency: f64,
    order: usize,
}

impl StreamFilter {
    pub const MIN_SAMPLES: usize = 16;

    pub fn new(cutoff_frequency: f64, order: usize) -> Self {
        Self {
            cutoff_frequency,
            order,
        }
    }

    pub fn apply(&self, stream: &mut RecordedStream) -> Result<()> {
        let n_samples = stream.timestamps.len();

        if n_samples < Self::MIN_SAMPLES {
            tracing::debug!("Not enough samples to filter safely, skipping");
            return Ok(());
        }

        let duration = stream
            .timestamps
            .last()
            .map(|&t| f32::from(t))
            .wrap_err("Insufficient data in stream")?;

        let sample_rate = n_samples as f64 / duration as f64;

        tracing::trace!(
            "Applying Butterworth ({:.1} Hz) (est. rate: {sample_rate:.0} Hz, {n_samples} over {duration:.1} s)",
            self.cutoff_frequency
        );

        let filter = Filter::new(
            self.order,
            sample_rate,
            Cutoff::LowPass(self.cutoff_frequency),
        )?;

        let mut buffer = Vec::with_capacity(stream.n_channels * n_samples);

        for channel in 0..stream.n_channels {
            buffer.clear();

            buffer.extend(Self::iter_channel(stream, channel).map(|t| *t as f64));

            let new_data = filter.bidirectional(&buffer)?;

            for (old, new) in Self::iter_channel(stream, channel).zip(new_data) {
                *old = new as f32;
            }
        }

        Ok(())
    }

    fn iter_channel(stream: &mut RecordedStream, offset: usize) -> impl Iterator<Item = &mut f32> {
        stream
            .channel_data
            .iter_mut()
            .skip(offset)
            .step_by(stream.n_channels)
    }
}

/* == LoadTransform == */

#[derive(Debug, Clone, Default)]
pub struct LoadTransform {
    rotation: Rotation3<f32>,
    translation: Vector3<f32>,
}

impl LoadTransform {
    pub fn new(point: &Point) -> Self {
        let o = &point.orientation;

        Self {
            rotation: Rotation3::from_euler_angles(o.x, o.y, o.z),
            translation: point.position,
        }
    }

    pub fn apply(&self, load: &Load) -> Load {
        let force = self.rotation * load.force;
        let moment = self.rotation * load.moment - self.translation.cross(&force);
        Load { force, moment }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_transform() {
        let point = Point {
            position: Vector3::new(0., 0., 1.),
            orientation: Vector3::zeros(),
        };

        let load = Load {
            force: Vector3::new(1., 0., 0.),
            moment: Vector3::new(0., 1., 0.),
        };

        let transform = LoadTransform::new(&point);

        assert_eq!(
            transform.apply(&load),
            Load {
                force: Vector3::new(1., 0., 0.),
                moment: Vector3::zeros(),
            }
        );
    }
}
