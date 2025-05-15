use std::iter::{self, Peekable};

use eyre::Result;
use polars::prelude::*;

use super::{
    experiment::{RecordedStream, Run, RunSample, RunSampleIterator},
    sink::StreamInfo,
};

#[derive(Debug, Clone, Copy)]
pub struct SegmentationOptions {
    method: SegmentationMethod,
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

    pub fn rows<'a>(&self, run: &'a Run) -> Option<(SegmentedRunRows<'a>, Box<[f32]>)> {
        let time = self.time_iterator(run)?;

        let n_channels = run.total_channels();
        let buffer = vec![0.0; n_channels].into_boxed_slice();

        Some((SegmentedRunRows::new(run, time), buffer))
    }

    pub fn columns(&self, run: &Run) -> Option<SegmentedRunColumns> {
        let (mut rows, mut buffer) = self.rows(run)?;

        let n_samples = rows.time.total;
        let mut time = Vec::with_capacity(n_samples);

        let streams = (0..buffer.len())
            .map(|_| Vec::with_capacity(n_samples))
            .collect();

        while let Some(data) = rows.next(&mut buffer) {
            time.push(data);
        }

        Some(SegmentedRunColumns { time, streams })
    }

    fn time_iterator(&self, run: &Run) -> Option<TimeIterator> {
        let run_duration = f32::from(run.duration()?);

        let (duration, count) = match self.method {
            SegmentationMethod::Count(count) => (run_duration, count),
            SegmentationMethod::Frequency(freq) => {
                let count = (run_duration * freq).floor() as u32;
                let duration = count as f32 / freq;

                (duration, count)
            }
        };

        Some(TimeIterator::new(duration, count))
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
        // Get a mutable reference to the cursor and compute its timestamp
        let Some(cursor) = self.cursor.as_mut() else {
            return false;
        };

        let cursor_time = f32::from(cursor.time);

        // The cursor can't go backwards (RunSample iterator is uni-directional)
        assert!(time >= cursor_time, "Time went backwards");

        // Keep advancing the cursor until we find the closest pre-time sample
        while let Some(next_sample) = self.iter_samples.peek() {
            if f32::from(next_sample.time) > time {
                break;
            }

            *cursor = unsafe { self.iter_samples.next().unwrap_unchecked() };
        }

        // Peek at the next sample, or return the current sample if we exhausted the iterator
        let Some(next_cursor) = self.iter_samples.peek() else {
            data.copy_from_slice(cursor.channel_data);
            return true;
        };

        // Interpolate between the two samples, clamping the time value (no extrapolation)
        let lt = cursor_time;
        let rt = f32::from(next_cursor.time);
        let t = (time - lt) / (rt - lt);

        if t.is_nan() {
            data.copy_from_slice(cursor.channel_data);
            return true;
        }

        // Don't extrapolate beyond the first or last sample
        let t = t.clamp(0., 1.);

        // Compute the channel interpolation and return buffer
        let l = cursor.channel_data.iter();
        let r = next_cursor.channel_data.iter();
        let o = data.iter_mut();

        for ((&l, &r), o) in l.zip(r).zip(o) {
            *o = l + (r - l) * t;
        }

        true
    }
}

impl SegmentedRunColumns {
    pub fn dataframe(self, streams: &[StreamInfo]) -> Result<DataFrame> {
        let time_column =
            ChunkedArray::<Float32Type>::from_vec("time".into(), self.time).into_column();

        let channel_columns = self
            .streams
            .into_iter()
            .zip(streams.iter().flat_map(|s| s.qualified_channel_names()))
            .map(|(iter, name)| {
                ChunkedArray::<Float32Type>::from_vec(name.into(), iter).into_column()
            });

        let columns = iter::once(time_column).chain(channel_columns).collect();

        Ok(DataFrame::new(columns)?)
    }
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
