use std::time::Duration;

use clap::Parser;
use eyre::Result;
use nalgebra::Vector3;
use tokio::time::sleep;

use crate::{
    config::{Config, HardwareConfig},
    data::{experiment::Run, processing::StreamFilter, sink::DataSink},
    defs::{Load, PoseEuler},
    hardware::{
        HardwareContext,
        load_cell::LoadCell,
        robot_arm::{Motion, MotionDiscriminants},
    },
    misc::data::deinterleave_data,
};

const G: f32 = 9.80665;

#[derive(Clone, Debug, Parser)]
pub struct MeasureOpts {
    #[clap(short, long, default_value = "config.yaml")]
    config: String,

    #[clap(long, default_value_t = Measure::MEASUREMENT_TIME_S)]
    sample_duration: f32,

    #[clap(long, default_value_t = 10.)]
    cutoff_frequency: f32,
}

pub struct Measure;

impl Measure {
    const POSES: [PoseEuler; 3] = [
        PoseEuler::new(400., 50., 300., 0., 0., 0.),
        PoseEuler::new(400., 50., 300., 0., 90., 0.),
        PoseEuler::new(400., 50., 300., 90., 0., 0.),
    ];

    const OFFSET_X: f32 = 175.;
    const MEASUREMENT_TIME_S: f32 = 0.5;

    pub async fn run(opts: MeasureOpts) -> Result<()> {
        let config = Config::load(&opts.config).await?;

        let config = HardwareConfig {
            robot_arm: config.hardware.robot_arm,
            load_cell: config.hardware.load_cell,
            ..Default::default()
        };

        let mut context = HardwareContext::builder().build(&config).await?;
        let mut builder = DataSink::builder();

        for agent in context.iter_mut() {
            agent.register(&mut builder).await;
            agent.start().await;
        }

        let (sink, streams) = builder.build();

        let robot_arm = context.robot_arm.as_mut().unwrap();
        let controller = robot_arm.controller();

        controller.go_home(MotionDiscriminants::Direct).await?;
        controller.wait_settled().await?;

        controller
            .set_offset(PoseEuler {
                x: Self::OFFSET_X,
                ..Default::default()
            })
            .await?;

        let mut runs: Vec<Run> = Vec::with_capacity(Self::POSES.len());

        for pose in Self::POSES {
            controller.move_to(Motion::Linear(pose)).await?;
            controller.wait_settled().await?;

            sink.start_recording().await;
            sleep(Duration::from_secs_f32(opts.sample_duration)).await;
            let run = sink.stop_recording().await;

            runs.push(run);
        }

        controller.go_home(MotionDiscriminants::Direct).await?;
        controller.wait_settled().await?;

        let mut results = Vec::with_capacity(runs.len());

        for mut run in runs {
            let mut stream = run.get_stream_mut(LoadCell::NAME, &streams).unwrap();

            StreamFilter::new(opts.cutoff_frequency as f64).apply(&mut stream)?;

            let channels: [_; 6] =
                deinterleave_data(&stream.channel_data, LoadCell::CHANNELS.len())
                    .into_vec()
                    .try_into()
                    .unwrap();

            let channel_means = channels.map(|channel| {
                let samples = channel.len() as f64;
                let sum = channel.into_iter().map(f64::from).sum::<f64>();
                (sum / samples) as f32
            });

            let [fx, fy, fz, tx, ty, tz] = channel_means;

            results.push(Load {
                force: Vector3::new(fx, fy, fz),
                moment: Vector3::new(tx, ty, tz),
            })
        }

        let loads: [Load; 3] = results.try_into().unwrap();

        let bias = solve_bias(loads);
        let (mg, o) = solve_resultant_force(loads, bias);

        let m = mg / G;

        println!("Force:  {mg} N ({m} kg)");
        println!("Offset: {o} m");
        println!("Bias:   {} N, {} Nm", bias.force, bias.moment);

        Ok(())
    }
}

/// Calculates the constant force and moment bias vector of the load cell.
///
/// Force bias components are determined by averaging values from orientations
/// where they are isolated from the applied load. Moment bias components are
/// determined from orientations where they are directly measured.
fn solve_bias(loads: [Load; 3]) -> Load {
    let fx = loads[0].force.x + loads[2].force.x;
    let fy = loads[0].force.y + loads[1].force.y;
    let fz = loads[1].force.z + loads[2].force.z;

    let mx = loads[1].moment.x;
    let my = loads[2].moment.y;
    let mz = loads[0].moment.z;

    let force = Vector3::new(fx, fy, fz) / 2.;
    let moment = Vector3::new(mx, my, mz);

    Load::new(force, moment)
}

/// Calculates the gravitational force magnitude (mg) on the object and its
/// center of mass offset vector (o) relative to the load cell origin.
///
/// It uses the raw measurements from three orientations and the pre-calculated
/// sensor bias. The mg value is an average of three derivations. Each component
/// of o is an average of two derivations.
fn solve_resultant_force(loads: [Load; 3], bias: Load) -> (f32, Vector3<f32>) {
    let mg0 = bias.force.z - loads[0].force.z;
    let mg1 = loads[1].force.x - bias.force.x;
    let mg2 = bias.force.y - loads[2].force.y;

    let mg = (mg0 + mg1 + mg2) / 3.;

    let mgo_x = (loads[0].moment.y - bias.moment.y) + (bias.moment.z - loads[2].moment.z);
    let mgo_y = (bias.moment.x - loads[0].moment.x) + (bias.moment.z - loads[1].moment.z);
    let mgo_z = (loads[1].moment.y - bias.moment.y) + (loads[2].moment.x - bias.moment.x);

    let o = Vector3::new(mgo_x, mgo_y, mgo_z) / (2. * mg);

    (mg, o)
}
