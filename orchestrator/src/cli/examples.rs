use std::{io, sync::Arc, time::Duration};

use eyre::Result;
use tokio::{task::JoinSet, time::sleep};

use crate::{
    hardware::example_counter::ExampleCounter,
    misc::plot_juggler::PlotJugglerBroadcaster,
    recording::{RecordedSample, Sink, StreamDefinition},
};

pub async fn counter() -> Result<()> {
    let sink = Sink::new();

    let mut mock0 = ExampleCounter::new();
    mock0.subscribe(&sink, "counter0".to_owned()).await?;

    let mut mock1 = ExampleCounter::new();
    mock1.subscribe(&sink, "counter1".to_owned()).await?;

    let null_handle = sink
        .add_stream(
            "null".to_owned(),
            ["null0", "null1"].map(str::to_owned).to_vec(),
        )
        .await?;

    sink.clear_buffer().await;
    sink.set_time_now().await;

    // This will not get recorded
    null_handle.write_now(&[0., f32::NAN]).await;

    sink.set_record(true);

    // This will get recorded
    null_handle.write_now(&[1., f32::NAN]).await;

    sleep(Duration::from_secs_f32(1.2)).await;

    sink.set_record(false);

    let recording = sink.complete().await;
    recording.encode(&mut io::stdout())?;

    Ok(())
}

pub async fn plot_juggler() -> Result<()> {
    let plot = Arc::new(PlotJugglerBroadcaster::create(None, None)?);

    let mock_streams = Arc::new(["counter0", "counter1"].map(|name| StreamDefinition {
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
    streams: Arc<[StreamDefinition; 2]>,
) -> Result<()> {
    let stream_id_f32 = stream_id as f32 + 1.;

    sleep(Duration::from_secs_f32(0.5 * stream_id_f32)).await;

    for time in 0..1000 {
        let t = 1e-2 * (time as f32) + stream_id_f32;
        let v = t.sin() / stream_id_f32;

        let data = RecordedSample {
            channel_data: &[v],
            definition: &streams[stream_id],
            recording_timestamp_us: (1e5 * t) as u32,
        };

        plot.send(&data)?;

        sleep(Duration::from_micros(100)).await;
    }

    Ok(())
}
