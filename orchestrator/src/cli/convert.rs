use std::{io, path::PathBuf};

use clap::{Parser, ValueEnum};
use eyre::{ContextCompat, Result, eyre};
use polars::prelude::*;

use crate::data::{experiment::Experiment, session::SessionMetadata};

const DEFAULT_DIVISIONS: u32 = 100;

#[derive(Clone, Debug, Parser)]
pub struct ConvertOpts {
    #[clap(short, long)]
    pub path: PathBuf,

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
    let maybe_metadata = SessionMetadata::find(&opts.path).await;
    let experiment = Experiment::load(&opts.path).await?;

    let run = experiment
        .runs
        .get(opts.run as usize)
        .wrap_err_with(|| eyre!("Run {} not found", opts.run))?;

    let metadata = maybe_metadata.unwrap_or_else(|| experiment.guess_metadata());

    let mut df = run.dataframe(&metadata.streams, opts.divisions)?;

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
