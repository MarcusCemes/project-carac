use std::{io, time::Duration};

use eyre::Result;
use tokio::time::sleep;

use crate::{config::Config, hardware::HardwareContext, recording::Sink};

pub async fn launch(config_path: &str) -> Result<()> {
    let config = Config::load(config_path).await?;

    let ctx = HardwareContext::builder().build(&config.hardware).await?;
    let sink = Sink::new();

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

    sink.clear_buffer().await;
    sink.set_time_now().await;
    sink.set_record(true);

    sleep(Duration::from_secs_f32(2.)).await;

    sink.set_record(false);

    if let Some(lc) = &ctx.load_cell {
        lc.stop_streaming().await?;
    }

    let recording = sink.complete().await;
    recording.encode_writer(&mut io::stdout())?;

    Ok(())
}
