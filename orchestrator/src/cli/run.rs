use std::time::Duration;

use eyre::Result;
use tokio::time::sleep;

use crate::{config::Config, data::orchestrator::Orchestrator, hardware::HardwareContext};

const TIMEOUT: Duration = Duration::from_secs(3);

pub async fn launch(config_path: &str) -> Result<()> {
    let config = Config::load(config_path).await?;

    let context = HardwareContext::builder()
        .with_timeout(TIMEOUT)
        .build(&config.hardware)
        .await?;

    let mut orchestrator = Orchestrator::create(config, context).await?;

    orchestrator.create_experiment("test".to_owned()).await?;
    orchestrator.start().await;

    if let Some(lc) = &orchestrator.context().load_cell {
        lc.set_bias().await?;
        lc.start_streaming().await?;
    }

    orchestrator.start_run().await;

    sleep(Duration::from_secs_f32(2.)).await;

    orchestrator.finish_run().await?;
    orchestrator.finish_experiment().await;
    orchestrator.stop().await;

    // let metadata = ExperimentMetadata::new(Some("Test Run".to_string()), sink.streams().await);
    // let experiment = Experiment::new(metadata, vec![run]);

    // experiment.write(&mut io::stdout()).await?;

    Ok(())
}
