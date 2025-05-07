use std::mem;

use chrono::DateTime;
use eyre::Result;
use tokio::io;

use crate::data::experiment::Experiment;

pub async fn display_data() -> Result<()> {
    let experiment = Experiment::read(&mut io::stdin()).await?;

    let start = DateTime::from_timestamp_millis(experiment.metadata.timestamp_ms).unwrap();

    println!("# == Metadata == #\n");
    println!("Time     {start:?}");

    println!(
        "Name     {}",
        experiment
            .metadata
            .name
            .unwrap_or_else(|| "(unnamed)".to_string())
    );

    println!("Streams  {}", experiment.metadata.streams.len());
    println!("Runs     {}", experiment.runs.len());
    println!("\n\n# == Streams == #");

    for stream in &experiment.metadata.streams {
        println!("\n{}", stream.name);

        for channel in &stream.channels {
            println!("  - {channel}");
        }
    }

    for (i, run) in experiment.runs.iter().enumerate() {
        println!("\n\n# == Run {i} == #");

        for (stream, definition) in run
            .recorded_streams
            .iter()
            .zip(experiment.metadata.streams.iter())
        {
            let n_channels = definition.channels.len();
            let n_samples = stream.timestamps.len();
            let size = n_samples * n_channels * mem::size_of::<f32>();

            println!("\n  {} ({n_samples} - {size} B)", definition.name);

            for sample in stream.iter_samples() {
                println!(
                    "    - {:.03}: {:+.03?}",
                    sample.time_s(),
                    sample.channel_data
                );
            }
        }
    }

    Ok(())
}
