use std::io;

use clap::{Parser, ValueEnum};
use eyre::{ContextCompat, Result};
use polars::prelude::*;

use crate::data::experiment::Experiment;

const DEFAULT_DIVISIONS: u32 = 100;

#[derive(Clone, Debug, Parser)]
pub struct ConvertOpts {
    #[clap(short, long, default_value = "csv")]
    pub format: OutputFormat,
    #[clap(short, long, default_value_t = DEFAULT_DIVISIONS)]
    pub divisions: u32,
    #[clap(short, long, default_value_t = 0)]
    pub run: usize,
}

#[derive(Clone, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum OutputFormat {
    Csv,
    Parquet,
}

pub async fn segment(opts: ConvertOpts) -> Result<()> {
    let experiment = Experiment::read(&mut tokio::io::stdin()).await?;

    let run = experiment
        .runs
        .get(opts.run)
        .wrap_err("Invalid run index")?;

    let mut df = run.dataframe(&experiment.metadata.streams, opts.divisions)?;

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
