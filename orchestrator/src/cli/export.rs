use std::{fs::create_dir, path::PathBuf};

use clap::{Parser, ValueEnum};
use eyre::{Result, bail};
use indicatif::{ProgressBar, ProgressStyle};
use polars::prelude::*;

use crate::data::{
    experiment::Experiment,
    processing::StreamFilter,
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

    #[clap(short, long)]
    pub cutoff_frequency: Option<f32>,

    #[clap(long, default_value_t = 1)]
    pub order: usize,
}

#[derive(Clone, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum OutputFormat {
    Csv,
    Parquet,
}

pub fn export(opts: ExportOpts) -> Result<()> {
    let Ok(metadata) = SessionMetadata::load(&opts.session_path) else {
        bail!("Session metadata not found");
    };

    let extension = opts.format.extension();
    let output_dir = opts.session_path.join(OUTPUT_DIR);

    if !output_dir.exists() {
        create_dir(&output_dir)?;
    }

    let session = Session::open(opts.session_path, metadata.streams)?;
    let experiments = session.list_experiments()?;

    tracing::info!("Found {} experiments", experiments.len());

    let bar = ProgressBar::new(experiments.len() as u64);

    bar.set_style(
        ProgressStyle::with_template(
            "[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}",
        )
        .unwrap()
        .progress_chars("##-"),
    );

    let mut total_runs = 0;

    for (id, path) in experiments {
        bar.inc(1);

        let experiment = Experiment::load(&path)?;

        for (i, mut run) in experiment.runs.into_iter().enumerate() {
            if let Some(cutoff_frequency) = opts.cutoff_frequency {
                let filter = StreamFilter::new(cutoff_frequency as f64, opts.order);

                for stream in &mut run.recorded_streams {
                    let _ = filter.apply(stream);
                }
            }

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

            total_runs += 1;
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
