use std::path::{self, PathBuf};

use clap::Parser;
use eyre::{Result, bail};
use indicatif::{ProgressBar, ProgressStyle};
use plotters::prelude::*;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use tokio::fs;

use crate::data::{
    experiment::Experiment,
    plot::create_scatter_plot,
    processing::StreamFilter,
    session::{Session, SessionMetadata},
    sink::StreamInfo,
};

const HEIGHT: u32 = 800;
const WIDTH: u32 = 1800;

const PLOTS_DIR: &str = "plots";

#[derive(Clone, Debug, Parser)]
pub struct PlotOpts {
    session_path: PathBuf,

    #[clap(short, long)]
    output_path: Option<PathBuf>,

    #[clap(short, long)]
    cutoff_frequency: Option<f32>,

    #[clap(long, default_value_t = 1)]
    order: usize,

    #[clap(long)]
    only_streams: Option<Vec<String>>,
}

pub async fn plot(opts: PlotOpts) -> Result<()> {
    let Ok(metadata) = SessionMetadata::load(&opts.session_path).await else {
        bail!("Session metadata not found");
    };

    let output_dir = opts
        .output_path
        .unwrap_or_else(|| opts.session_path.join(PLOTS_DIR));

    if !output_dir.exists() {
        fs::create_dir(&output_dir).await?;
    }

    let session = Session::open(opts.session_path, metadata.streams).await?;
    let experiments = session.list_experiments().await?;

    tracing::info!("Found {} experiments", experiments.len());

    let filter = opts
        .cutoff_frequency
        .map(|f| StreamFilter::new(f as f64, opts.order));
    let streams = &session.metadata().streams;

    let bar = ProgressBar::new(0);

    bar.set_style(
        ProgressStyle::with_template(
            "[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}",
        )
        .unwrap()
        .progress_chars("##-"),
    );

    experiments.into_par_iter().try_for_each(|(id, path)| {
        render_experiment(
            id,
            &path,
            &output_dir,
            &opts.only_streams,
            &filter,
            streams,
            &bar,
        )
    })?;

    bar.finish_and_clear();

    Ok(())
}

fn render_experiment(
    id: u32,
    path: &path::Path,
    output_dir: &path::Path,
    only_streams: &Option<Vec<String>>,
    filter: &Option<StreamFilter>,
    stream_info: &[StreamInfo],
    bar: &ProgressBar,
) -> Result<()> {
    let (experiment, experiment_dir) = load_experiment(id, path, output_dir)?;

    let jobs = experiment.runs.len() * stream_info.iter().map(|s| s.channels.len()).sum::<usize>();
    bar.inc_length(jobs as u64);

    for (i, run) in experiment.runs.into_iter().enumerate() {
        let plot_name = format!("Experiment #{id} ({}) - Run #{i}", experiment.header.name);

        for (mut stream, info) in run.recorded_streams.into_iter().zip(stream_info.iter()) {
            if let Some(streams) = only_streams {
                if !streams.iter().any(|s| info.name.starts_with(s)) {
                    bar.inc(1);
                    continue;
                }
            }

            if let Some(filter) = filter {
                let _ = filter.apply(&mut stream);
            }

            let time = stream.time_column();
            let columns = stream.columns();

            for (column, column_name) in columns
                .into_iter()
                .zip(info.qualified_channel_names().into_iter())
            {
                let filename = format!("{i:02}_{}.png", column_name.replace('/', "-"));
                let path = experiment_dir.join(filename);

                create_and_save_plot(&time, column, &plot_name, &column_name, &path)?;
                bar.inc(1);
            }
        }
    }

    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn load_experiment(
    id: u32,
    path: &path::Path,
    output_dir: &path::Path,
) -> Result<(Experiment, PathBuf)> {
    let experiment = Experiment::load(path).await?;
    let experiment_dir = output_dir.join(format!("{id:04}"));

    if !fs::try_exists(&experiment_dir).await? {
        fs::create_dir(&experiment_dir).await?;
    }

    Ok((experiment, experiment_dir))
}

fn create_and_save_plot(
    time: &[f32],
    values: Box<[f32]>,
    chart_name: &str,
    axis_name: &str,
    filename: &std::path::Path,
) -> Result<()> {
    if time.is_empty() {
        return Ok(());
    }

    let backend = BitMapBackend::new(filename, (WIDTH, HEIGHT)).into_drawing_area();

    backend.fill(&WHITE)?;

    create_scatter_plot(&backend, (time, &values), chart_name, axis_name)?;

    backend.present()?;

    Ok(())
}
