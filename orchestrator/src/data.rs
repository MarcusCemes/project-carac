use std::{iter::Peekable, ops::RangeInclusive};

use crate::recording::{RecordedSample, RecordedStream, Recording, StreamDefinition};

struct StreamInterpolator<'a> {
    channels: Vec<f32>,
    cursor: RecordedSample<'a>,
    iterator: Peekable<Box<dyn Iterator<Item = RecordedSample<'a>> + 'a>>,
    recorded_stream: &'a RecordedStream,
}

impl<'a> StreamInterpolator<'a> {
    fn new(recorded_stream: &'a RecordedStream) -> Option<Self> {
        // Iterate over the stream's samples
        let mut iterator = recorded_stream.iter_samples();

        // Get the first sample and initialise the interpolated channels buffer
        let cursor = iterator.next()?;
        let channels = Vec::with_capacity(recorded_stream.definition.n_channels());

        // Box the iterator to allow for dynamic dispatch (complex type)
        let iterator: Box<dyn Iterator<Item = RecordedSample>> = Box::new(iterator);
        let iterator = iterator.peekable();

        Some(Self {
            cursor,
            channels,
            iterator,
            recorded_stream,
        })
    }

    fn next(&mut self, time_s: f32) -> &[f32] {
        // Keep advancing the cursor until we find the closest pre-time sample
        while let Some(next_sample) = self.iterator.peek() {
            if next_sample.timestamp_s() > time_s {
                break;
            }

            self.cursor = self.iterator.next().unwrap();
        }

        // Peek at the next sample, or return the current sample if we exhausted the iterator
        let Some(next_cursor) = self.iterator.peek() else {
            return self.cursor.channel_data;
        };

        // Interpolate between the two samples, clamping the time value (no extrapolation)
        let lt = self.cursor.timestamp_s();
        let rt = next_cursor.timestamp_s();
        let t = (time_s - lt) / (rt - lt);

        if t.is_nan() {
            return self.cursor.channel_data;
        }

        // Don't extrapolate beyond the first or last sample
        let t = t.clamp(0., 1.);

        self.channels.clear();

        // Compute the channel interpolation and return buffer
        let l = self.cursor.channel_data.iter();
        let r = next_cursor.channel_data.iter();

        for (&l, &r) in l.zip(r) {
            self.channels.push(l + (r - l) * t);
        }

        &self.channels
    }
}

pub struct SegmentedRecordingIterator<'a> {
    definitions: Vec<&'a StreamDefinition>,
    interpolators: Vec<StreamInterpolator<'a>>,
    time_iterator: SegmentedTimeIterator,
}

impl<'a> SegmentedRecordingIterator<'a> {
    pub fn new(recording: &'a Recording, divisions: u32) -> Option<Self> {
        let mut start = u32::MAX;
        let mut end = u32::MIN;

        // Try to create an interpolator for each stream (must have at least two samples)
        let mut definitions = Vec::new();

        let interpolators = recording
            .recorded_streams
            .iter()
            .filter_map(|stream| {
                start = start.min(*stream.data_timestamps_us.first()?);
                end = end.max(*stream.data_timestamps_us.last()?);
                definitions.push(&stream.definition);

                StreamInterpolator::new(stream)
            })
            .collect();

        // Convert microseconds to seconds
        let [start, end] = [start, end].map(|v| 1e-6 * v as f32);

        let time_iterator = SegmentedTimeIterator::new(start, end, divisions);

        Some(Self {
            definitions,
            interpolators,
            time_iterator,
        })
    }

    pub fn next(&mut self, buf: &mut Vec<f32>) -> Option<f32> {
        let time_s = self.time_iterator.next()?;

        buf.clear();

        for interpolator in &mut self.interpolators {
            buf.extend_from_slice(interpolator.next(time_s));
        }

        Some(time_s)
    }

    pub fn definitions(&self) -> &[&StreamDefinition] {
        &self.definitions
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
