use std::{io, path::PathBuf};

use clap::{Parser, ValueEnum};
use eyre::{Context, ContextCompat, Result};
use polars::prelude::*;

use crate::data::{session::Session, sink::StreamInfo};

const DEFAULT_DIVISIONS: u32 = 100;

#[derive(Clone, Debug, Parser)]
pub struct ConvertOpts {
    #[clap(short, long)]
    pub path: PathBuf,

    #[clap(short, long)]
    pub experiment: u32,

    #[clap(short, long, default_value_t = 0)]
    pub run: u32,

    #[clap(short, long, default_value = "csv")]
    pub format: OutputFormat,

    #[clap(short, long, default_value_t = DEFAULT_DIVISIONS)]
    pub divisions: u32,
}

#[derive(Clone, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum OutputFormat {
    Csv,
    Parquet,
}

pub async fn segment(opts: ConvertOpts) -> Result<()> {
    let metadata = Session::read_metadata(&opts.path)
        .await
        .inspect_err(|_| tracing::warn!("Failed to read session metadata"))
        .ok();

    let experiment = Session::read_experiment(&opts.path, opts.experiment)
        .await
        .wrap_err("Experiment not found")?;

    let run = experiment
        .runs
        .get(opts.run as usize)
        .wrap_err("Run not found")?;

    let streams = StreamInfo::use_or(metadata.as_deref(), &experiment.header.streams);

    let mut df = run.dataframe(&streams, opts.divisions)?;

    match opts.format {
        OutputFormat::Csv => {
            CsvWriter::new(&mut io::stdout()).finish(&mut df)?;
        }

        OutputFormat::Parquet => {
            ParquetWriter::new(&mut io::stdout()).finish(&mut df)?;
        }
    }

    Ok(())
}
