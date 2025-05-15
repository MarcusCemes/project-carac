use eyre::Result;

use crate::{
    config::Config,
    data::orchestrator::Orchestrator,
    defs::PoseEuler,
    hardware::{
        HardwareContext,
        robot_arm::{Motion, MotionDiscriminants, SpeedProfile},
    },
    misc::sleep,
};

pub async fn launch(config_path: &str) -> Result<()> {
    let config = Config::load(config_path).await?;

    let context = HardwareContext::builder().build(&config.hardware).await?;

    let mut orchestrator = Orchestrator::create(config, context).await?;

    orchestrator.new_experiment("Test".to_owned()).await?;
    orchestrator.start().await;

    if let Some(lc) = &orchestrator.context().load_cell {
        lc.set_bias().await?;
        lc.start_streaming().await?;
    }

    experiment_code(&mut orchestrator).await?;

    // let metadata = ExperimentMetadata::new(Some("Test Run".to_string()), sink.streams().await);
    // let experiment = Experiment::new(metadata, vec![run]);

    // experiment.write(&mut io::stdout()).await?;

    if let Some(lc) = &orchestrator.context().load_cell {
        lc.stop_streaming().await?;
    }

    Ok(())
}

async fn experiment_code(orchestrator: &mut Orchestrator) -> Result<()> {
    let mut profile = SpeedProfile {
        acceleration_scale: 100,
        deceleration_scale: 100,
        velocity_scale: 100,
        rotation_limit: 45.,
        translation_limit: 10000.,
    };

    let offset = PoseEuler {
        x: 660.,
        z: 0.,
        ..Default::default()
    };

    let mut pos = PoseEuler {
        x: 600.,
        z: 600.,
        ..Default::default()
    };

    if let Some(robot) = &orchestrator.context().robot_arm {
        let mut c = robot.controller();
        c.go_home(MotionDiscriminants::Linear).await?;
        c.wait_settled().await?;
    }

    if let Some(wind) = &mut orchestrator.context().wind_shape {
        wind.enable_power().await?;
        wind.request_control().await;

        // wind.set_fan_speed(46).await?;
        wind.set_fan_speed(10).await?;
        sleep(2.).await;
    }

    if let Some(robot) = &orchestrator.context().robot_arm {
        let mut c = robot.controller();

        c.set_profile(profile).await?;
        c.set_offset(offset).await?;

        pos.rx = 90.;
        c.move_to(Motion::Linear(pos)).await?;
        c.wait_settled().await?;
    }

    orchestrator.new_run().await;

    if let Some(robot) = &orchestrator.context().robot_arm {
        let mut c = robot.controller();

        pos.rx = -90.;
        profile.rotation_limit = 180.;
        c.set_profile(profile).await?;

        c.move_to(Motion::Linear(pos)).await?;
        c.wait_settled().await?;
    }

    orchestrator.save_run().await?;
    orchestrator.new_run().await;

    if let Some(robot) = &orchestrator.context().robot_arm {
        let mut c = robot.controller();

        pos.rx = 90.;

        c.move_to(Motion::Linear(pos)).await?;
        c.wait_settled().await?;
    }

    orchestrator.save_run().await?;
    orchestrator.save_experiment().await?;

    sleep(5.).await;

    if let Some(wind) = &mut orchestrator.context().wind_shape {
        wind.set_fan_speed(0).await?;
        wind.release_control().await;
        wind.disable_power().await?;
    }

    if let Some(robot) = &orchestrator.context().robot_arm {
        let mut c = robot.controller();

        profile.rotation_limit = 45.;
        c.set_profile(profile).await?;

        c.go_home(MotionDiscriminants::Linear).await?;
        c.wait_settled().await?;
    }

    orchestrator.stop().await;

    Ok(())
}
