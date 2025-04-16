use std::{iter::Peekable, ops::RangeInclusive};

use crate::recorder::Recording;

struct StreamInterpolator<'a> {
    channels: Box<[f32]>,
    cursor: usize,
    iterator: Peekable<Box<dyn Iterator<Item = usize> + 'a>>,
    recording: &'a Recording,
}

impl<'a> StreamInterpolator<'a> {
    fn new(recording: &'a Recording, stream_id: u8) -> Option<Self> {
        let n_channels = recording.streams[stream_id as usize].channels.len();

        // Construct an iterator over the samples of the specified stream
        let mut iterator = recording
            .samples
            .iter()
            .enumerate()
            .filter_map(move |(i, s)| (s.stream_id == stream_id).then_some(i));

        // Get the first sample and initialise the interpolated channels buffer
        let cursor = iterator.next()?;
        let channels = vec![0.0; n_channels].into_boxed_slice();

        // Box the iterator to allow for dynamic dispatch (complex type)
        let iterator: Box<dyn Iterator<Item = usize>> = Box::new(iterator);
        let iterator = iterator.peekable();

        Some(Self {
            cursor,
            channels,
            iterator,
            recording,
        })
    }

    fn next(&mut self, time: f32) -> &[f32] {
        // Advance the cursor until we find the closest left-side sample
        while let Some(&next_cursor) = self.iterator.peek() {
            let next_sample = &self.recording.samples[next_cursor];

            if next_sample.time as f32 <= time {
                self.cursor = next_cursor;
                self.iterator.next();
            } else {
                break;
            }
        }

        let l = self.recording.get_at(self.cursor);

        // Peek at the next sample, or return the current sample if we exhausted the iterator
        let Some(r) = self
            .iterator
            .peek()
            .map(|&next_cursor| self.recording.get_at(next_cursor))
        else {
            return l.data;
        };

        // Interpolate between the two samples, clamping the time value (no extrapolation)
        let lt = l.sample.time as f32;
        let rt = r.sample.time as f32;
        let t = (time - lt) / (rt - lt);

        if t.is_nan() {
            return l.data;
        }

        let t = t.clamp(0., 1.);

        // Compute the channel interpolation and return buffer
        for (i, (&l, &r)) in l.data.iter().zip(r.data).enumerate() {
            self.channels[i] = l + (r - l) * t;
        }

        &self.channels
    }
}

pub struct SegmentedRecordingIterator<'a> {
    stream_ids: Vec<u8>,
    streams: Vec<StreamInterpolator<'a>>,
    time_iter: SegmentedTimeIterator,
    total_channels: usize,
}

impl<'a> SegmentedRecordingIterator<'a> {
    pub fn new(recording: &'a Recording, divisions: u32) -> Option<Self> {
        let first_sample = recording.samples.first()?;
        let last_sample = recording.samples.last().unwrap();
        let total_channels = recording.n_channels();

        // List of stream_ids corresponding to each interpolation
        let mut stream_ids = Vec::new();

        // Try to create an interpolator for ech stream (must have at least two samples)
        let streams = (0..recording.streams.len() as u8)
            .flat_map(|id| StreamInterpolator::new(recording, id).inspect(|_| stream_ids.push(id)))
            .collect::<Vec<_>>();

        let time_iter = SegmentedTimeIterator::new(
            first_sample.time as f32,
            last_sample.time as f32,
            divisions,
        );

        Some(Self {
            streams,
            stream_ids,
            time_iter,
            total_channels,
        })
    }

    pub fn streams(&self) -> &[u8] {
        &self.stream_ids
    }
}

impl Iterator for SegmentedRecordingIterator<'_> {
    type Item = Vec<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        let time = self.time_iter.next()?;

        let mut buffer = Vec::with_capacity(self.total_channels);

        for stream in &mut self.streams {
            let channels = stream.next(time);
            buffer.extend_from_slice(channels);
        }

        Some(buffer)
    }
}

struct SegmentedTimeIterator {
    index: u32,
    divisions: u32,
    range: RangeInclusive<f32>,
}

impl SegmentedTimeIterator {
    fn new(start_time: f32, end_time: f32, divisions: u32) -> Self {
        debug_assert!(divisions > 1, "Divisions must be greater than 1");

        Self {
            index: 0,
            divisions,
            range: start_time..=end_time,
        }
    }
}

impl Iterator for SegmentedTimeIterator {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        (self.index < self.divisions).then(|| {
            let r = self.index as f32 / (self.divisions - 1) as f32;
            let time = self.range.start() + r * (self.range.end() - self.range.start());
            self.index += 1;
            time
        })
    }
}
