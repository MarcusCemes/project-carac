use std::{net::IpAddr, sync::Arc, time::Duration};

use clap::{Parser, Subcommand};
use eyre::{Context, Result};
use tokio::{io, task::JoinSet, time::sleep};

use orchestrator::{
    data::{
        experiment::{Experiment, ExperimentMetadata},
        run::{RunSample, StreamInfo},
        sink::DataSink,
    },
    defs::Point,
    hardware::{
        example_counter::ExampleCounter,
        robot_arm::{Motion, MotionDiscriminants, RobotArm, SpeedProfile},
        wind_shape::WindShape,
        HardwareAgent,
    },
    misc::plot_juggler::PlotJugglerBroadcaster,
};

#[derive(Parser)]
struct Opts {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    Counter,
    PlotJuggler,

    RobotArm(RobotArmOpts),

    WindShape(WindShapeOpts),
}

#[tokio::main(flavor = "current_thread")]
pub async fn main() -> Result<()> {
    orchestrator::init()?;

    let opts = Opts::parse();

    match opts.command {
        Command::Counter => counter().await,
        Command::PlotJuggler => plot_juggler().await,
        Command::RobotArm(opts) => robot_arm(opts).await,
        Command::WindShape(opts) => wind_shape(opts).await,
    }
}

/* == Counter == */

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

/* == PlotJuggler == */

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

        set.spawn(plot_juggler_task(stream_id, plot, streams));
    }

    set.join_all().await;

    Ok(())
}

async fn plot_juggler_task(
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

/* == RobotArm == */

#[derive(Parser)]
struct RobotArmOpts {
    #[arg(long)]
    ip: IpAddr,

    #[arg(long, default_value_t = 20000)]
    port: u16,
}

async fn robot_arm(opts: RobotArmOpts) -> Result<()> {
    tracing::info!("Connecting to robot arm");

    let mut robot_arm = RobotArm::connect(opts.ip, opts.port)
        .await
        .wrap_err("Failed to connect to RobotArm")?;

    robot_arm.on_record().await;

    let mut r = robot_arm.controller();

    let mut point = Point {
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

    let mut profile = SpeedProfile {
        rotation_limit: 10000.,
        translation_limit: 10000.,
        acceleration_scale: 20,
        deceleration_scale: 20,
        velocity_scale: 100,
    };

    r.set_offset(offset).await?;
    r.set_profile(profile).await?;

    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    point.x = 1200.;
    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    offset.z = 100.;
    profile.acceleration_scale = 100;
    profile.deceleration_scale = 100;

    r.set_offset(offset).await?;
    r.set_profile(profile).await?;

    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    point.rx = 90.;
    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    point.rx = -90.;
    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    point.rx = 0.;
    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    offset.z = 0.;
    r.set_offset(offset).await?;

    point.x = 500.;
    point.z = 450.;
    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    profile.acceleration_scale = 15;
    profile.deceleration_scale = 15;
    profile.rotation_limit = 30.;
    r.set_profile(profile).await?;

    point.rz = -45.;
    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    point.rz = 20.;
    r.move_to(Motion::Linear(point)).await?;
    r.wait_settled().await?;

    profile.rotation_limit = 10000.;
    profile.translation_limit = 10000.;

    r.go_home(MotionDiscriminants::Direct).await?;
    r.wait_settled().await?;

    robot_arm.on_pause().await;

    Ok(())
}

/* == WindShape == */

#[derive(Parser)]
struct WindShapeOpts {
    #[arg(long)]
    ip: IpAddr,
}

async fn wind_shape(opts: WindShapeOpts) -> Result<()> {
    let mut wind_shape = WindShape::connect(opts.ip)
        .await
        .expect("Failed to connect to WindShape");

    wind_shape.request_control().await;

    wind_shape.enable_power().await?;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(10).await?;
    sleep_s(3.).await;

    wind_shape.set_fan_speed(15).await?;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(20).await?;
    sleep_s(1.).await;

    wind_shape.set_fan_speed(5).await?;
    sleep_s(1.).await;

    wind_shape.disable_power().await?;
    wind_shape.release_control().await;

    Ok(())
}

/* == Utils == */

async fn sleep_s(seconds: f32) {
    tokio::time::sleep(Duration::from_secs_f32(seconds)).await;
}
