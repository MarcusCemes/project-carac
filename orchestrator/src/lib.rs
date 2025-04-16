#![allow(dead_code)]

use std::{io, sync::Arc, time::Duration};

use bincode::config;
use hardware::{
    example_counter::ExampleCounter, load_cell::LoadCell, motion_capture::MotionCapture,
};
use tokio::{task::JoinSet, time::sleep};

use crate::{
    defs::*,
    hardware::{
        robot_arm::{Motion, MotionConfig, RobotArm},
        wind_shape::WindShape,
    },
    misc::plot_juggler::PlotJugglerBroadcaster,
    recorder::{Recorder, Recording},
};

mod data;
mod defs;
mod hardware;
mod misc;
mod recorder;

pub async fn convert(divisions: u32) -> io::Result<()> {
    let recording: Recording = bincode::decode_from_std_read(&mut io::stdin(), binary_config())
        .map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                "Failed to decode recording from stdin",
            )
        })?;

    let segmented_recording = recording
        .segment(divisions)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "Failed to segment recording"))?;

    let mut w = csv::Writer::from_writer(io::stdout());

    let header = segmented_recording
        .streams()
        .iter()
        .map(|&i| &recording.streams[i as usize])
        .flat_map(|stream| {
            stream
                .channels
                .iter()
                .map(|c| format!("{}/{}", stream.name, c))
        })
        .collect::<Vec<_>>();

    w.write_record(&header)?;

    for values in segmented_recording {
        let record = values.iter().map(|v| v.to_string());
        w.write_record(record)?;
    }

    Ok(())
}

pub async fn plot_juggler_demo() -> io::Result<()> {
    let plot = Arc::new(PlotJugglerBroadcaster::new(None));

    let streams = Arc::new([
        recorder::Stream {
            name: "counter0".to_string(),
            channels: vec!["count".to_string()],
        },
        recorder::Stream {
            name: "counter1".to_string(),
            channels: vec!["count".to_string()],
        },
    ]);

    let mut set = JoinSet::new();

    for stream_id in 0..streams.len() {
        let plot = plot.clone();
        let streams = streams.clone();

        set.spawn(async move {
            let stream_id_f32 = stream_id as f32 + 1.;

            sleep_s(0.5 * stream_id_f32).await;

            for time in 0..1000 {
                let t = 2e-2 * stream_id_f32 * time as f32 + stream_id_f32;
                let v = t.sin() / stream_id_f32;

                plot.send(
                    &recorder::Measurement {
                        data: &[v],
                        sample: &recorder::Sample {
                            data_index: 0,
                            stream_id: stream_id as u8,
                            time,
                        },
                    },
                    &*streams,
                );

                sleep_s(1e-4).await;
            }
        });
    }

    set.join_all().await;

    Ok(())
}

pub async fn counter() -> io::Result<()> {
    let recorder = Recorder::new();

    let mut mock0 = ExampleCounter::new();
    mock0.subscribe(&recorder, "counter0").await;

    let mut mock1 = ExampleCounter::new();
    mock1.subscribe(&recorder, "counter1").await;

    recorder.add_stream("null", &["null0", "null1"]).await;

    tracing::info!("Starting recording...");
    recorder.new_recording().await;
    recorder.start_recording();

    sleep_s(1.2).await;

    recorder.stop_recording();

    let recording = recorder.commit().await;

    tracing::info!("Streams: {:?}", recording.streams);
    tracing::info!("Samples: {:?}", recording.samples);
    tracing::info!("Data: {:?}", recording.sample_data);

    bincode::encode_into_std_write(recording, &mut io::stdout(), binary_config()).unwrap();

    Ok(())
}

pub async fn run() -> io::Result<()> {
    let ctx = Context::create().await;
    let recorder = Recorder::new();

    ctx.motion_capture.subscribe("Tapper", &recorder).await;
    ctx.load_cell.subscribe(&recorder).await;

    ctx.load_cell.set_bias().await?;
    ctx.load_cell.start_streaming().await?;

    tracing::info!("Starting recording...");
    recorder.new_recording().await;
    recorder.start_recording();

    sleep_s(1.2).await;

    recorder.stop_recording();
    ctx.load_cell.stop_streaming().await?;

    let recording = recorder.commit().await;

    bincode::encode_into_std_write(recording, &mut io::stdout(), binary_config()).unwrap();

    Ok(())
}

struct Context {
    motion_capture: MotionCapture,
    load_cell: LoadCell,
    wind_shape: WindShape,
}

impl Context {
    async fn create() -> Self {
        Context {
            motion_capture: MotionCapture::connect(None)
                .await
                .expect("Failed to connect to motion capture"),

            load_cell: LoadCell::connect(None)
                .await
                .expect("Failed to connect to load cell"),

            wind_shape: WindShape::connect(None)
                .await
                .expect("Failed to connect to WindShape"),
        }
    }
}

#[allow(dead_code)]
pub async fn test_robot_arm() {
    tracing::info!("Connecting to robot arm");

    let mut robot_arm = RobotArm::connect(None, None, None)
        .await
        .expect("Failed to connect to RobotArm");

    let mut position = Point {
        x: 500.,
        y: 100.,
        z: 400.,
        rx: 0.,
        ry: 0.,
        rz: 0.,
    };

    let mut offset = Point {
        x: 660.,
        z: 20.,
        ..Default::default()
    };

    let mut config = MotionConfig {
        rotation: 10000.,
        translation: 10000.,
        acceleration_scale: 20,
        deceleration_scale: 20,
        speed_scale: 100,
    };

    robot_arm.set_offset(&offset).await;
    robot_arm.set_config(&config).await;

    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.x = 1200.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    offset.z = 100.;
    config.acceleration_scale = 100;
    config.deceleration_scale = 100;

    robot_arm.set_offset(&offset).await;
    robot_arm.set_config(&config).await;

    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rx = 90.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rx = -90.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rx = 0.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    offset.z = 0.;
    robot_arm.set_offset(&offset).await;

    position.x = 500.;
    position.z = 450.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    config.acceleration_scale = 15;
    config.deceleration_scale = 15;
    config.rotation = 30.;
    robot_arm.set_config(&config).await;

    position.rz = -45.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    position.rz = 20.;
    robot_arm.move_to(&Motion::Linear(position)).await;
    robot_arm.wait_motion().await;

    config.rotation = 10000.;
    config.translation = 10000.;

    robot_arm.return_home().await;
    robot_arm.wait_motion().await;
}

#[allow(dead_code)]
pub async fn test_wind_shape() {
    let mut wind_shape = WindShape::connect(None)
        .await
        .expect("Failed to connect to WindShape");

    wind_shape.request_control().await;

    wind_shape.enable_power().await;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(10).await;
    sleep_s(3.).await;

    wind_shape.set_fan_speed(15).await;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(20).await;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(5).await;
    sleep_s(1.).await;

    wind_shape.disable_power().await;
    wind_shape.release_control().await;
}

async fn sleep_s(seconds: f32) {
    sleep(Duration::from_secs_f32(seconds)).await;
}

fn binary_config() -> impl config::Config {
    config::standard()
        .with_little_endian()
        .with_fixed_int_encoding()
}
