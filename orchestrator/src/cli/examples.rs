use std::{sync::Arc, time::Duration};

use eyre::Result;
use tokio::{io, task::JoinSet, time::sleep};

use crate::{
    data::{
        experiment::{Experiment, ExperimentMetadata},
        run::{RunSample, StreamInfo},
        sink::DataSink,
    },
    hardware::example_counter::ExampleCounter,
    misc::plot_juggler::PlotJugglerBroadcaster,
};

pub async fn counter() -> Result<()> {
    let sink = DataSink::new();

    let mut mock0 = ExampleCounter::new();
    mock0.subscribe(&sink, "counter0".to_owned()).await?;

    let mut mock1 = ExampleCounter::new();
    mock1.subscribe(&sink, "counter1".to_owned()).await?;

    let null_handle = sink
        .add_stream(
            "null".to_owned(),
            vec!["null0".to_owned(), "null1".to_owned()],
        )
        .await?;

    sink.clear().await;

    // This will not get recorded
    null_handle.add(&[0., f32::NAN]).await;

    tracing::info!("Recording...");
    sink.set_record(true).await;

    // This will get recorded
    null_handle.add(&[1., f32::NAN]).await;

    sleep(Duration::from_secs_f32(1.6)).await;

    tracing::info!("Stopping recording...");
    sink.set_record(false).await;

    let run = sink.finish().await;

    let experiment = Experiment::new(
        ExperimentMetadata::new(Some("Test Run".to_string()), sink.streams().await),
        vec![run],
    );

    experiment.write(&mut io::stdout()).await?;

    Ok(())
}

pub async fn plot_juggler() -> Result<()> {
    let plot = Arc::new(PlotJugglerBroadcaster::create(None, None)?);

    let mock_streams = Arc::new(["counter0", "counter1"].map(|name| StreamInfo {
        name: name.to_string(),
        channels: vec!["count".to_string()],
    }));

    let mut set = JoinSet::new();

    for stream_id in 0..mock_streams.len() {
        let plot = plot.clone();
        let streams = mock_streams.clone();

        set.spawn(plot_task(stream_id, plot, streams));
    }

    set.join_all().await;

    Ok(())
}

async fn plot_task(
    stream_id: usize,
    plot: Arc<PlotJugglerBroadcaster>,
    streams: Arc<[StreamInfo; 2]>,
) -> Result<()> {
    let stream_id_f32 = stream_id as f32 + 1.;

    sleep(Duration::from_secs_f32(0.5 * stream_id_f32)).await;

    for time in 0..1000 {
        let t = 1e-2 * (time as f32) + stream_id_f32;
        let v = t.sin() / stream_id_f32;

        let data = RunSample {
            channel_data: &[v],
            delta_us: (1e5 * t) as u32,
        };

        plot.send(&data, &*streams)?;

        sleep(Duration::from_micros(100)).await;
    }

    Ok(())
}
