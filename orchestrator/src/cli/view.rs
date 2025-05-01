use std::{io, mem};

use chrono::DateTime;
use eyre::Result;

use crate::recording::Recording;

pub fn display_data() -> Result<()> {
    let recording = Recording::decode_reader(&mut io::stdin())?;

    let start = DateTime::from_timestamp_micros(recording.start_timestamp_us).unwrap();
    let n_streams = recording.recorded_streams.len();

    println!("# == Metadata == #\n");
    println!("Time     {start:?}");
    println!("Streams  {n_streams}");
    println!("\n\n# == Data == #");

    for recorded_stream in &recording.recorded_streams {
        let name = &recorded_stream.definition.name;
        let n_channels = recorded_stream.definition.channels.len();

        println!("\n{name}\n");
        println!("  Channels");

        for channel in &recorded_stream.definition.channels {
            println!("    - {channel}");
        }

        let n_samples = recorded_stream.data_timestamps_us.len();
        let size = n_samples * n_channels * mem::size_of::<f32>();

        println!("\n  Data ({n_samples} - {size} B)");

        for sample in recorded_stream.iter_samples() {
            println!(
                "    - {:.03}: {:+.03?}",
                sample.timestamp_s(),
                sample.channel_data
            );
        }
    }

    Ok(())
}
