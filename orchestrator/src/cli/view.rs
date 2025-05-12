use std::{mem, path::PathBuf};

use clap::Parser;
use eyre::Result;

use crate::data::session::Session;

#[derive(Parser)]
pub struct ViewOpts {
    #[arg(short, long)]
    path: PathBuf,

    #[arg(short, long)]
    id: u32,
}

pub async fn view(opts: ViewOpts) -> Result<()> {
    let maybe_streams = Session::read_metadata(&opts.path).await.ok();
    let experiment = Session::read_experiment(&opts.path, opts.id).await?;

    println!("# == Metadata == #\n");

    println!("Name     {}", experiment.header.name);
    println!("Streams  {}", experiment.header.streams.len());
    println!("Runs     {}", experiment.runs.len());
    println!("\n\n# == Streams == #");

    if let Some(streams) = &maybe_streams {
        for stream in streams {
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
            .zip(experiment.stream_names(maybe_streams.as_deref()))
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
