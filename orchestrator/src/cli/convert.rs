use std::{
    fs::File,
    io::{self, Write},
    path::PathBuf,
};

use clap::{Parser, ValueEnum};
use eyre::{ContextCompat, Result, eyre};
use polars::prelude::*;

use crate::data::{experiment::Experiment, processing::StreamFilter, session::SessionMetadata};

const DEFAULT_DIVISIONS: u32 = 100;

#[derive(Clone, Debug, Parser)]
pub struct ConvertOpts {
    pub path: PathBuf,

    #[clap(short, long, default_value_t = 0)]
    pub run: u32,

    #[clap(short, long, default_value = "csv")]
    pub format: OutputFormat,

    #[clap(short, long, default_value_t = DEFAULT_DIVISIONS)]
    pub divisions: u32,

    #[clap(short, long)]
    cutoff_frequency: Option<f32>,

    #[clap(long, default_value_t = 1)]
    order: usize,

    #[clap(short, long)]
    pub output: Option<PathBuf>,
}

#[derive(Clone, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum OutputFormat {
    Csv,
    Parquet,
}

pub async fn segment(opts: ConvertOpts) -> Result<()> {
    let maybe_metadata = SessionMetadata::find(&opts.path).await;
    let mut experiment = Experiment::load(&opts.path).await?;

    let metadata = maybe_metadata.unwrap_or_else(|| experiment.guess_metadata());

    let run = experiment
        .runs
        .get_mut(opts.run as usize)
        .wrap_err_with(|| eyre!("Run {} not found", opts.run))?;

    if let Some(cutoff_frequency) = opts.cutoff_frequency {
        let filter = StreamFilter::new(cutoff_frequency as f64, opts.order);

        for stream in &mut run.recorded_streams {
            let _ = filter.apply(stream);
        }
    }

    let mut df = run.dataframe(&metadata.streams, opts.divisions)?;
    let mut output = get_output(opts.clone())?;

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

fn get_output(opts: ConvertOpts) -> Result<Box<dyn Write>> {
    Ok(match opts.output {
        Some(path) => Box::new(File::create(path)?),
        None => Box::new(io::stdout()),
    })
}
