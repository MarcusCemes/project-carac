use std::time::Duration;

use clap::Parser;
use eyre::Result;
use nalgebra::Vector3;
use tokio::time::sleep;

use crate::{
    config::{Config, HardwareConfig},
    data::{experiment::Run, processing::StreamFilter, sink::DataSink},
    defs::{G, Load, Point},
    hardware::{
        HardwareContext,
        load_cell::LoadCell,
        robot_arm::defs::{ArmConfig, Command as RC, Motion, MotionKind, Profile},
    },
    misc::data::deinterleave_data,
};

const CUTOFF_FREQUENCY: f32 = 5.;
#[derive(Clone, Debug, Parser)]
pub struct MeasureOpts {
    #[clap(short, long, default_value = "config.yaml")]
    config: String,

    #[clap(long, default_value_t = Measure::MEASUREMENT_TIME_S)]
    sample_duration: f32,

    #[clap(long, default_value_t = CUTOFF_FREQUENCY)]
    cutoff_frequency: f32,

    #[clap(long, default_value_t = 1)]
    order: usize,
}

pub struct Measure;

impl Measure {
    const POSES: [Point; 3] = [
        Point::new(400., 50., 300., 0., 0., 0.),
        Point::new(400., 50., 300., 0., 90., 0.),
        Point::new(400., 50., 300., 90., 0., 0.),
    ];

    const OFFSET_X: f32 = 175.;
    const MEASUREMENT_TIME_S: f32 = 1.;

    pub async fn run(opts: MeasureOpts) -> Result<()> {
        let config = Config::load(&opts.config).await?;

        let config = HardwareConfig {
            robot_arm: config.hardware.robot_arm,
            load_cell: config.hardware.load_cell,
            ..Default::default()
        };

        let mut context = HardwareContext::builder().build(&config).await?;
        let mut builder = DataSink::builder();

        for agent in context.iter() {
            agent.register(&mut builder).await;
            agent.start().await;
        }

        let (sink, streams) = builder.build();

        let robot = context.robot_arm.as_mut().unwrap();

        for i in [
            RC::SetToolOffset(Point::ZERO),
            RC::SetConfig(ArmConfig::default()),
            RC::SetProfile(
                Profile::builder()
                    .with_translation(500.)
                    .with_rotation(90.)
                    .with_smoothing(30)
                    .build(),
            ),
            RC::ReturnHome(MotionKind::Direct),
        ] {
            robot.instruction(i).await?;
        }

        robot.try_wait_settled().await?;

        let point_offset = Point::new(Self::OFFSET_X, 0., 0., 0., 0., 0.);
        robot.instruction(RC::SetToolOffset(point_offset)).await?;

        let mut runs: Vec<Run> = Vec::with_capacity(Self::POSES.len());

        for pose in Self::POSES {
            tracing::info!("Moving to pose...");

            robot.instruction(RC::Move(Motion::Direct(pose))).await?;
            robot.try_wait_settled().await?;

            tracing::info!("Stabilising...");
            sleep(Duration::from_secs(2)).await;

            sink.start_recording().await;
            sleep(Duration::from_secs_f32(opts.sample_duration)).await;
            let run = sink.stop_recording().await;

            runs.push(run);
        }

        robot
            .instruction(RC::ReturnHome(MotionKind::Direct))
            .await?;

        robot.try_wait_settled().await?;

        let mut results = Vec::with_capacity(runs.len());

        for mut run in runs {
            let stream = run.get_stream_mut(LoadCell::NAME, &streams).unwrap();

            StreamFilter::new(opts.cutoff_frequency as f64, opts.order).apply(stream)?;

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
            });
        }

        let loads: [Load; 3] = results.try_into().unwrap();

        println!("Raw loads:");

        for (i, load) in loads.iter().enumerate() {
            println!("{i}: {:?} {:?}", load.force, load.moment);
        }

        let bias = solve_bias(loads);
        let (mg, o) = solve_resultant_force(loads, bias);

        let m = mg / G;

        println!(
            "\nForce:  {mg} N ({m} kg)\nOffset [m]: {o:?}\nForce bias [N]: {:?}\nMoment bias [Nm]: {:?}",
            bias.force, bias.moment
        );

        Ok(())
    }
}

/// Calculates the constant force and moment bias vector of the load cell.
///
/// Force bias components are determined by averaging values from orientations
/// where they are isolated from the applied load. Moment bias components are
/// determined from orientations where they are directly measured.
fn solve_bias(loads: [Load; 3]) -> Load {
    let fx = (loads[0].force.x + loads[1].force.x) / 2.;
    let fy = (loads[0].force.y + loads[2].force.y) / 2.;
    let fz = (loads[1].force.z + loads[2].force.z) / 2.;

    let mx = loads[2].moment.x;
    let my = loads[1].moment.y;
    let mz = loads[0].moment.z;

    let force = Vector3::new(fx, fy, fz);
    let moment = Vector3::new(mx, my, mz);

    Load { force, moment }
}

/// Calculates the gravitational force magnitude (mg) on the object and its
/// center of mass offset vector (o) relative to the load cell origin.
///
/// It uses the raw measurements from three orientations and the pre-calculated
/// sensor bias. The mg value is an average of three derivations. Each component
/// of o is an average of two derivations.
fn solve_resultant_force(loads: [Load; 3], bias: Load) -> (f32, Vector3<f32>) {
    let mg0 = bias.force.z - loads[0].force.z;
    let mg1 = bias.force.y - loads[1].force.y;
    let mg2 = bias.force.x - loads[2].force.x;

    let mg = if mg0.signum() == mg1.signum() && mg1.signum() == mg2.signum() {
        (mg0 + mg1 + mg2) / 3.
    } else {
        tracing::warn!(
            "Measured force signs are inconsistent! Loads may not be correctly transformed."
        );
        (mg0.abs() + mg1.abs() + mg2.abs()) / 3.
    };

    let mgox = (loads[0].moment.y - bias.moment.y) + (bias.moment.z - loads[1].moment.z);
    let mgoy = (bias.moment.x - loads[0].moment.x) + (loads[2].moment.z - bias.moment.z);
    let mgoz = (loads[1].moment.x - bias.moment.x) + (bias.moment.y - loads[2].moment.y);

    let o = Vector3::new(mgox, mgoy, mgoz) / (2. * mg);

    (mg, o)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::defs::{G, Load};

    use super::*;

    #[test]
    fn test_solution() {
        const MASS: f32 = 2.5;

        let mg = MASS * G;
        let offset = Vector3::new(0.05, -0.02, 0.10);

        let bias = Load {
            force: Vector3::new(0.1, -0.05, 0.2),
            moment: Vector3::new(-0.04, 0.06, -0.03),
        };

        let loads = gravity_vectors(MASS).map(|f| generate_load(bias, offset, f));

        let calculated_bias = solve_bias(loads);

        assert_vec_approx_eq(calculated_bias.force, bias.force, 1e-5);
        assert_vec_approx_eq(calculated_bias.moment, bias.moment, 1e-5);

        let (calculated_mg, calculated_offset) = solve_resultant_force(loads, calculated_bias);

        assert!((calculated_mg - mg).abs() < 1e-5);
        assert_vec_approx_eq(calculated_offset, offset, 1e-5);
    }

    fn gravity_vectors(mass: f32) -> [Vector3<f32>; 3] {
        let mg = mass * G;

        [
            Vector3::new(0.0, 0.0, -mg),
            Vector3::new(0.0, -mg, 0.0),
            Vector3::new(-mg, 0.0, 0.0),
        ]
    }

    fn generate_load(bias: Load, offset: Vector3<f32>, gravity: Vector3<f32>) -> Load {
        let m_gravity = offset.cross(&gravity);

        Load {
            force: bias.force + gravity,
            moment: bias.moment + m_gravity,
        }
    }

    fn assert_vec_approx_eq(a: Vector3<f32>, b: Vector3<f32>, epsilon: f32) {
        assert!(
            (a - b).norm() < epsilon,
            "Vector assertion failed: {a:?} != {b:?} within tolerance {epsilon}",
        );
    }
}
