use std::path::{self, PathBuf};

use clap::Parser;
use eyre::{Result, bail};
use indicatif::{ProgressBar, ProgressStyle};
use plotters::prelude::*;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use tokio::fs;

use crate::data::{
    experiment::Experiment,
    processing::StreamFilter,
    session::{Session, SessionMetadata},
    sink::StreamInfo,
};

const PLOTS_DIR: &str = "plots";

#[derive(Clone, Debug, Parser)]
pub struct PlotOpts {
    session_path: PathBuf,

    #[clap(short, long)]
    output_path: Option<PathBuf>,

    #[clap(short, long)]
    cutoff_frequency: Option<f32>,

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

    let filter = opts.cutoff_frequency.map(|f| StreamFilter::new(f as f64));
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

                create_scatter_plot(&time, column, &plot_name, &column_name, &path)?;
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

fn create_scatter_plot(
    time: &[f32],
    values: Box<[f32]>,
    name: &str,
    axis: &str,
    filename: &std::path::Path,
) -> Result<()> {
    if time.is_empty() {
        return Ok(());
    }

    let root = BitMapBackend::new(filename, (1800, 800)).into_drawing_area();

    root.fill(&WHITE)?;

    let n_samples = time.len();
    let t_min = *time.first().unwrap();
    let t_max = *time.last().unwrap();

    let v_min = values.iter().copied().fold(f32::INFINITY, f32::min);
    let v_max = values.iter().copied().fold(f32::NEG_INFINITY, f32::max);

    let mut chart = ChartBuilder::on(&root)
        .caption(name, ("sans-serif", 24))
        .margin(24)
        .x_label_area_size(64)
        .y_label_area_size(64)
        .build_cartesian_2d(t_min..t_max, v_min..v_max)?;

    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc(axis)
        .x_label_style(("sans-serif", 16))
        .y_label_style(("sans-serif", 16))
        .draw()?;

    let (opacity, size) = match n_samples {
        0..500 => (1.0, 4),
        500..1000 => (0.5, 2),
        _ => (0.25, 1),
    };

    let style = ShapeStyle {
        color: RGBAColor(149, 81, 150, opacity),
        filled: true,
        stroke_width: 0,
    };

    let series = time
        .into_iter()
        .copied()
        .zip(values.into_iter())
        .map(|coords| Circle::new(coords, size, style));

    chart.draw_series(series)?;

    root.present()?;

    Ok(())
}
