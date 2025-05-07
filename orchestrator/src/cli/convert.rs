use std::io;

use eyre::{ContextCompat, Result};

use crate::data::{experiment::Experiment, run::StreamInfo};

pub async fn segment(divisions: u32, run: usize) -> Result<()> {
    let experiment = Experiment::read(&mut tokio::io::stdin()).await?;

    let run = experiment.runs.get(run).wrap_err("Invalid run index")?;

    let mut segmented_run = run
        .segment(&experiment.metadata.streams, divisions)
        .wrap_err("Failed to segment recording")?;

    let mut w = csv::Writer::from_writer(io::stdout());

    let header = experiment
        .metadata
        .streams
        .iter()
        .flat_map(StreamInfo::qualified_channel_names)
        .collect::<Vec<_>>();

    w.write_field("time")?;
    w.write_record(&header)?;

    let mut buf = Vec::new();

    while let Some(time) = segmented_run.next(&mut buf) {
        w.write_field(time.to_string())?;
        w.write_record(buf.iter().map(f32::to_string))?;
    }

    Ok(())
}
