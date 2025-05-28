use std::{net::IpAddr, sync::Arc};

use clap::{Parser, Subcommand};
use eyre::{Context, Result};
use tokio::{task::JoinSet, time::Instant};

use drone_lab::{
    data::sink::{DataSink, StreamInfo},
    defs::Point,
    hardware::{
        HardwareAgent,
        example_counter::ExampleCounter,
        robot_arm::{
            RobotArm,
            defs::Command as RC,
            defs::{Motion, MotionKind, Profile, ProfileLimit, ProfileScale},
        },
        wind_shape::WindShape,
    },
    misc::{plot_juggler::PlotJugglerBroadcaster, sleep},
};

macro_rules! movel {
    ($robot:expr, $point:expr) => {
        $robot.instruction(RC::Move(Motion::Linear($point))).await?;
    };
}

macro_rules! settle {
    ($robot:expr) => {
        $robot.try_wait_settled().await?;
    };
}

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
    drone_lab::init()?;
    drone_lab::banner();

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
    let mut builder = DataSink::builder();

    let mut mock0 = ExampleCounter::new("counter0".to_owned());
    mock0.register(&mut builder).await;

    let mut mock1 = ExampleCounter::new("counter1".to_owned());
    mock1.register(&mut builder).await;

    let null_handle = builder
        .register_stream(
            "null".to_owned(),
            ["null0", "null1"].map(str::to_owned).to_vec(),
        )
        .await;

    let (sink, _streams) = builder.build();

    if let Ok(broadcaster) = PlotJugglerBroadcaster::builder().build() {
        sink.set_broadcaster(broadcaster).await;
    }

    // This will not get recorded
    let now = Instant::now();
    null_handle.add(now, &[0., f32::NAN]).await;

    tracing::info!("Recording...");
    sink.start_recording().await;

    // This will get recorded
    let now = Instant::now();
    null_handle.add(now, &[1., f32::NAN]).await;

    sleep(1.6).await;

    tracing::info!("Stopping recording...");
    let _run = sink.stop_recording().await;

    Ok(())
}

/* == PlotJuggler == */

pub async fn plot_juggler() -> Result<()> {
    let plot = Arc::new(PlotJugglerBroadcaster::builder().build()?);

    let mock_streams = Arc::new(["counter0", "counter1"].map(|name| StreamInfo {
        name: name.to_string(),
        channels: ["count"].map(str::to_owned).to_vec(),
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

    sleep(0.5 * stream_id_f32).await;

    for time in 0..1000 {
        let t = 1e-2 * (time as f32) + stream_id_f32;
        let v = t.sin() / stream_id_f32;

        plot.send(t, &[v], &streams[stream_id])?;

        sleep(0.1).await;
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

    let mut robot = RobotArm::try_new(opts.ip, opts.port, None)
        .await
        .wrap_err("Failed to connect to RobotArm")?;

    robot.start().await;

    let mut point = Point::position(500., 100., 400.);
    let mut offset = Point::position(660., 20., 0.);

    let mut profile = Profile::builder().with_smoothing(20).build();

    robot.instruction(RC::SetToolOffset(offset)).await?;
    robot.instruction(RC::SetProfile(profile)).await?;
    movel!(robot, point);
    settle!(robot);

    point.position.x = 1200.;
    movel!(robot, point);
    settle!(robot);

    offset.position.z = 100.;
    profile.scale = ProfileScale::default();

    robot.instruction(RC::SetToolOffset(offset)).await?;
    robot.instruction(RC::SetProfile(profile)).await?;

    movel!(robot, point);
    settle!(robot);

    point.orientation.x = 90.;
    movel!(robot, point);
    settle!(robot);

    point.orientation.x = -90.;
    movel!(robot, point);
    settle!(robot);

    point.orientation.x = 0.;
    movel!(robot, point);
    settle!(robot);

    offset.position.z = 0.;
    robot.instruction(RC::SetToolOffset(offset)).await?;

    point.position.x = 500.;
    point.position.x = 500.;
    point.position.z = 450.;
    movel!(robot, point);
    settle!(robot);

    profile.scale.acceleration = 15;
    profile.scale.deceleration = 15;
    profile.limit.rotation = 30.;
    robot.instruction(RC::SetProfile(profile)).await?;

    point.orientation.z = -45.;
    movel!(robot, point);
    settle!(robot);

    point.orientation.z = 20.;
    movel!(robot, point);
    settle!(robot);

    profile.limit = ProfileLimit::default();

    robot
        .instruction(RC::ReturnHome(MotionKind::Direct))
        .await?;

    settle!(robot);

    robot.stop().await;

    Ok(())
}

/* == WindShape == */

#[derive(Parser)]
struct WindShapeOpts {
    #[arg(long)]
    ip: IpAddr,
}

async fn wind_shape(opts: WindShapeOpts) -> Result<()> {
    let mut wind_shape = WindShape::connect(opts.ip).await?;

    wind_shape.request_control().await;

    wind_shape.set_powered(true).await?;
    sleep(1.).await;

    wind_shape.set_fan_speed(0.1).await?;
    sleep(3.).await;

    wind_shape.set_fan_speed(0.15).await?;
    sleep(1.).await;

    wind_shape.set_fan_speed(0.2).await?;
    sleep(1.).await;

    wind_shape.set_fan_speed(0.05).await?;
    sleep(1.).await;

    wind_shape.set_powered(false).await?;
    wind_shape.release_control().await;

    Ok(())
}
