use std::time::Duration;

use eyre::Result;
use tokio::{io, time::sleep};

use crate::{
    config::Config,
    data::{
        experiment::{Experiment, ExperimentMetadata},
        sink::DataSink,
    },
    hardware::HardwareContext,
};

pub async fn launch(config_path: &str) -> Result<()> {
    let config = Config::load(config_path).await?;

    let ctx = HardwareContext::builder().build(&config.hardware).await?;
    let sink = DataSink::new();

    if let Some((mc, cfg)) = &ctx.motion_capture.zip(config.hardware.motion_capture) {
        for rb in &cfg.rigid_bodies {
            mc.subscribe(rb, &sink).await?;
        }
    }

    if let Some(lc) = &ctx.load_cell {
        lc.subscribe(&sink).await?;
        lc.set_bias().await?;
        lc.start_streaming().await?;
    }

    sink.clear().await;
    sink.set_record(true).await;

    sleep(Duration::from_secs_f32(2.)).await;

    sink.set_record(false).await;

    if let Some(lc) = &ctx.load_cell {
        lc.stop_streaming().await?;
    }

    let run = sink.finish().await;

    let metadata = ExperimentMetadata::new(Some("Test Run".to_string()), sink.streams().await);
    let experiment = Experiment::new(metadata, vec![run]);

    experiment.write(&mut io::stdout()).await?;

    Ok(())
}
