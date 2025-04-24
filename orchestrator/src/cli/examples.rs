use std::{io, sync::Arc, time::Duration};

use tokio::{task::JoinSet, time::sleep};

use crate::{
    hardware::example_counter::ExampleCounter,
    misc::plot_juggler::PlotJugglerBroadcaster,
    recording::{RecordedSample, Recorder, StreamDefinition},
};

pub async fn counter() -> io::Result<()> {
    let recorder = Recorder::new();

    let mut mock0 = ExampleCounter::new();
    mock0.subscribe(&recorder, "counter0").await;

    let mut mock1 = ExampleCounter::new();
    mock1.subscribe(&recorder, "counter1").await;

    recorder.add_stream("null", &["null0", "null1"]).await;

    recorder.clear_buffer().await;
    recorder.reset_reference_time().await;
    recorder.start_recording();

    sleep(Duration::from_secs_f32(1.2)).await;

    recorder.stop_recording();

    let recording = recorder.complete().await;
    eprintln!("\nRecording data:\n");

    for stream in &recording.recorded_streams {
        eprintln!("  Stream {}", stream.definition.name);

        for sample in stream.iter_samples() {
            eprintln!(
                "    {:.03}: {:+.03?}",
                sample.timestamp_s(),
                sample.channel_data
            )
        }

        eprintln!();
    }

    recording.encode(&mut io::stdout())?;

    Ok(())
}

pub async fn plot_juggler() -> io::Result<()> {
    let plot = Arc::new(PlotJugglerBroadcaster::new(None));

    let streams = Arc::new(["counter0", "counter1"].map(|name| StreamDefinition {
        name: name.to_string(),
        channels: vec!["count".to_string()],
    }));

    let mut set = JoinSet::new();

    for stream_id in 0..streams.len() {
        let plot = plot.clone();
        let streams = streams.clone();

        set.spawn(async move {
            let stream_id_f32 = stream_id as f32 + 1.;

            // Induce a delay to simulate different stream start times
            sleep(Duration::from_secs_f32(0.5 * stream_id_f32)).await;

            for time in 0..1000 {
                let t = 1e-2 * (time as f32) + stream_id_f32;
                let v = t.sin() / stream_id_f32;

                plot.send(&RecordedSample {
                    channel_data: &[v],
                    definition: &streams[stream_id],
                    recording_timestamp_us: (1e5 * t) as u32,
                });

                sleep(Duration::from_micros(100)).await;
            }
        });
    }

    set.join_all().await;

    Ok(())
}
