use std::{
    fs::File,
    io::{self, Write},
    path::PathBuf,
};

use clap::{Parser, ValueEnum};
use eyre::{ContextCompat, Result, eyre};
use polars::prelude::*;

use crate::data::{experiment::Experiment, processing::StreamFilter, session::SessionMetadata};

#[derive(Clone, Debug, Parser)]
pub struct ExtractOpts {
    pub path: PathBuf,

    pub stream: String,

    #[clap(short, long, default_value_t = 0)]
    pub run: u32,

    #[clap(long)]
    pub channel: Option<String>,

    #[clap(long)]
    pub skip: Option<u32>,

    #[clap(short, long, default_value = "csv")]
    pub format: OutputFormat,

    #[clap(short, long)]
    pub cutoff_frequency: Option<f32>,

    #[clap(long, default_value_t = 1)]
    pub order: usize,

    #[clap(short, long)]
    pub output: Option<PathBuf>,
}

#[derive(Clone, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum OutputFormat {
    Csv,
    Parquet,
}

pub fn extract(opts: ExtractOpts) -> Result<()> {
    let maybe_metadata = SessionMetadata::find(&opts.path);
    let mut experiment = Experiment::load(&opts.path)?;

    let metadata = maybe_metadata.unwrap_or_else(|| experiment.guess_metadata());

    let run = experiment
        .runs
        .get_mut(opts.run as usize)
        .wrap_err_with(|| eyre!("Run {} not found", opts.run))?;

    let Some(stream) = run.get_stream_mut(&opts.stream, &metadata.streams) else {
        eyre::bail!("Stream '{}' not found", opts.stream);
    };

    let stream_info = metadata
        .streams
        .iter()
        .find(|s| s.name == opts.stream)
        .unwrap();

    if let Some(cutoff_frequency) = opts.cutoff_frequency {
        let filter = StreamFilter::new(cutoff_frequency as f64, opts.order);
        let _ = filter.apply(stream);
    }

    let mut df = stream.dataframe(stream_info)?;
    let mut output = get_output(opts.clone())?;

    if let Some(channel) = opts.channel {
        df = df.select(["time", &channel])?;
    }

    if let Some(skip) = opts.skip {
        let indices: Vec<_> = (0..df.height() as u32).step_by(skip as usize).collect();
        let idx = IdxCa::new("idx".into(), &indices);

        df = df.take(&idx)?;
    }

    match opts.format {
        OutputFormat::Csv => {
            CsvWriter::new(&mut output).finish(&mut df)?;
        }

        OutputFormat::Parquet => {
            ParquetWriter::new(&mut output).finish(&mut df)?;
        }
    }

    Ok(())
}

fn get_output(opts: ExtractOpts) -> Result<Box<dyn Write>> {
    Ok(match opts.output {
        Some(path) => Box::new(File::create(path)?),
        None => Box::new(io::stdout()),
    })
}
