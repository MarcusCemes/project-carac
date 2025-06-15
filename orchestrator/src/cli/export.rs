use std::{
    fs::create_dir,
    path::{Path, PathBuf},
    sync::atomic::{AtomicUsize, Ordering},
};

use clap::Parser;
use eyre::{Result, bail};
use indicatif::{ProgressBar, ProgressStyle};
use polars::prelude::*;
use rayon::prelude::*;

use crate::{
    cli::common::OutputFormat,
    data::{
        experiment::{Experiment, Run},
        processing::StreamFilter,
        session::{Session, SessionMetadata},
        sink::StreamInfo,
    },
};

const DEFAULT_DIVISIONS: u32 = 100;
const OUTPUT_DIR: &str = "output";

#[derive(Clone, Debug, Parser)]
pub struct ExportOpts {
    pub session_path: PathBuf,

    #[clap(short, long)]
    pub run: Option<u16>,

    #[clap(short, long, default_value = "csv")]
    pub format: OutputFormat,

    #[clap(short, long, default_value_t = DEFAULT_DIVISIONS)]
    pub divisions: u32,

    #[clap(short, long)]
    pub cutoff_frequency: Option<f32>,

    #[clap(long, default_value_t = 1)]
    pub order: usize,
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

    let session = Session::open(opts.session_path.clone(), metadata.streams)?;
    let experiments = session.list_experiments()?;
    let streams = &session.metadata().streams;

    tracing::info!("Found {} experiments", experiments.len());

    let bar = ProgressBar::new(experiments.len() as u64);
    bar.set_style(
        ProgressStyle::with_template(
            "[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}",
        )
        .unwrap()
        .progress_chars("##-"),
    );

    let total_runs = AtomicUsize::new(0);

    experiments
        .into_par_iter()
        .try_for_each(|(id, path)| -> Result<()> {
            let experiment = Experiment::load(&path)?;

            if let Some(run_index) = opts.run {
                if let Some(run) = experiment.runs.into_iter().nth(run_index as usize) {
                    let experiment_name = path.file_name().unwrap().to_str().unwrap();
                    let run_name = format!("{}.{}", experiment_name, extension);
                    let run_path = output_dir.join(run_name);

                    process_and_save_run(run, &streams, &opts, &run_path)?;
                    total_runs.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                for (i, run) in experiment.runs.into_iter().enumerate() {
                    let run_name = Session::output_name(id, i, extension);
                    let run_path = output_dir.join(run_name);

                    process_and_save_run(run, streams, &opts, &run_path)?;
                    total_runs.fetch_add(1, Ordering::Relaxed);
                }
            }

            bar.inc(1);
            Ok(())
        })?;

    bar.finish_and_clear();
    tracing::info!("Exported {} runs", total_runs.load(Ordering::Relaxed));

    Ok(())
}

fn process_and_save_run(
    mut run: Run,
    streams: &[StreamInfo],
    opts: &ExportOpts,
    output_path: &Path,
) -> Result<()> {
    if let Some(cutoff_frequency) = opts.cutoff_frequency {
        let filter = StreamFilter::new(cutoff_frequency as f64, opts.order);
        for stream in &mut run.recorded_streams {
            let _ = filter.apply(stream);
        }
    }

    let mut df = run.dataframe(streams, opts.divisions)?;

    // Create the output file and write the DataFrame in the specified format
    let mut file = std::fs::File::create(output_path)?;
    match opts.format {
        OutputFormat::Csv => {
            CsvWriter::new(&mut file).finish(&mut df)?;
        }
        OutputFormat::Parquet => {
            ParquetWriter::new(&mut file).finish(&mut df)?;
        }
    }

    Ok(())
}
