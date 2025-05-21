use std::path::PathBuf;

use clap::{Parser, ValueEnum};
use eyre::{Result, bail};
use indicatif::ProgressBar;
use polars::prelude::*;
use tokio::fs;

use crate::data::{
    experiment::Experiment,
    session::{Session, SessionMetadata},
};

const DEFAULT_DIVISIONS: u32 = 100;
const OUTPUT_DIR: &str = "output";

#[derive(Clone, Debug, Parser)]
pub struct ExportOpts {
    pub session_path: PathBuf,

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

pub async fn export(opts: ExportOpts) -> Result<()> {
    let Ok(metadata) = SessionMetadata::load(&opts.session_path).await else {
        bail!("Session metadata not found");
    };

    let extension = opts.format.extension();
    let output_dir = opts.session_path.join(OUTPUT_DIR);

    if !output_dir.exists() {
        fs::create_dir(&output_dir).await?;
    }

    let session = Session::open(opts.session_path, metadata.streams).await?;
    let experiments = session.list_experiments().await?;

    tracing::info!("Found {} experiments", experiments.len());

    let bar = ProgressBar::new(experiments.len() as u64);
    let mut total_runs = 0;

    for (id, path) in experiments {
        bar.inc(1);

        let experiment = Experiment::load(&path).await?;

        for (i, run) in experiment.runs.into_iter().enumerate() {
            total_runs += 1;

            let run_name = Session::output_name(id, i, extension);
            let run_path = output_dir.join(run_name);

            let mut file = std::fs::File::create(&run_path)?;
            let mut df = run.dataframe(&session.metadata().streams, opts.divisions)?;

            match opts.format {
                OutputFormat::Csv => {
                    CsvWriter::new(&mut file).finish(&mut df)?;
                }

                OutputFormat::Parquet => {
                    ParquetWriter::new(&mut file).finish(&mut df)?;
                }
            }
        }
    }

    bar.finish_and_clear();
    tracing::info!("Exported {} runs", total_runs);

    Ok(())
}

impl OutputFormat {
    pub fn extension(&self) -> &'static str {
        match self {
            OutputFormat::Csv => "csv",
            OutputFormat::Parquet => "parquet",
        }
    }
}
