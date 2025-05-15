use std::{mem, path::PathBuf};

use clap::Parser;
use eyre::Result;

use crate::data::{experiment::Experiment, session::SessionMetadata};

#[derive(Parser)]
pub struct ViewOpts {
    path: PathBuf,
}

pub async fn view(opts: ViewOpts) -> Result<()> {
    let experiment = Experiment::load(&opts.path).await?;
    let maybe_metadata = SessionMetadata::find(&opts.path).await;

    println!("# == Metadata == #\n");

    println!("Name     {}", experiment.header.name);
    println!("Streams  {}", experiment.header.streams.len());
    println!("Runs     {}", experiment.runs.len());
    println!("\n\n# == Streams == #");

    if let Some(metadata) = &maybe_metadata {
        for stream in &metadata.streams {
            println!("\n{}", stream.name);

            for channel in &stream.channels {
                println!("  - {channel}");
            }
        }
    }

    for (i, run) in experiment.runs.iter().enumerate() {
        println!("\n\n# == Run {i} == #");

        for ((stream, &n_channels), name) in run
            .recorded_streams
            .iter()
            .zip(experiment.header.streams.iter())
            .zip(experiment.stream_names(maybe_metadata.as_ref().map(|m| m.streams.as_slice())))
        {
            let n_samples = stream.timestamps.len();
            let size = n_samples * n_channels as usize * mem::size_of::<f32>();

            println!("\n  {name} ({n_samples} - {size} B)");

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
