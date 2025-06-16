from asyncio import get_event_loop, run, sleep
from math import sqrt

from carac.defs import DRONE_NO_ACTUATION
from rich.status import Status

from carac.helpers import rad_to_deg
from carac.motion import BaseTrajectoryGenerator, SineTrajectoryGenerator
from carac.prelude import *


# == Parameters == #

DURATION_S: int = 240

WIND_SPEED: float = 0.0
WING_L: float = -1  # -1 = closed, 0.5 = open
WING_R: float = 1  # 1 = closed, -0.5 = open
THROTTLE: float = -1

POINT_FREQUENCY = 10
PLOT = False

DEVICE = "drone"

LIMITS_S = {
    "x": (-200.0, 00.0),
    "y": (-200.0, 00.0),
    "z": (-200.0, 00.0),
    "rx": (-50.0, 50.0),
    "ry": (-20.0, 20.0),
    "rz": (-15.0, 15.0),
}

SINE_PARAMS_S = {
    "x": [
        {"amp": 120.0, "freq": 0.4},
        {"amp": 80, "freq": 1.2, "phase": 0.2},
    ],
    "y": [
        {"amp": 120.0, "freq": 0.9, "phase": 0.15},
        {"amp": 80, "freq": sqrt(1.3), "phase": 0.5},
    ],
    "z": [
        {"amp": 120.0, "freq": 0.6, "phase": 0.05},
        {"amp": 80, "freq": 0.3, "phase": 0.1},
    ],
    "rx": [
        {"amp": 35, "freq": 0.7},
        {"amp": 20, "freq": 0.5, "phase": 0.3},
    ],
    "ry": [
        {"amp": 25, "freq": 0.8},
        {"amp": 15, "freq": 0.6, "phase": 0.7},
    ],
    "rz": [
        {"amp": 15, "freq": 0.7, "phase": 0.2},
        {"amp": 8, "freq": sqrt(0.4), "phase": 0.8},
    ],
}


LIMITS_L = {
    "x": (-250.0, 00.0),
    "y": (-250.0, 00.0),
    "z": (-250.0, 00.0),
    "rx": (-60.0, 60.0),
    "ry": (-25.0, 25.0),
    "rz": (-20.0, 20.0),
}

SINE_PARAMS_L = {
    "x": [
        {"amp": 150.0, "freq": 0.4},
        {"amp": 80, "freq": 1.2, "phase": 0.2},
    ],
    "y": [
        {"amp": 150.0, "freq": 0.9, "phase": 0.15},
        {"amp": 80, "freq": sqrt(1.3), "phase": 0.5},
    ],
    "z": [
        {"amp": 150.0, "freq": 0.6, "phase": 0.05},
        {"amp": 80, "freq": 0.3, "phase": 0.1},
    ],
    "rx": [
        {"amp": 40, "freq": 0.7},
        {"amp": 25, "freq": 0.5, "phase": 0.3},
    ],
    "ry": [
        {"amp": 28, "freq": 0.8},
        {"amp": 17, "freq": 0.6, "phase": 0.7},
    ],
    "rz": [
        {"amp": 18, "freq": 0.7, "phase": 0.2},
        {"amp": 10, "freq": sqrt(0.4), "phase": 0.8},
    ],
}

LIMITS = LIMITS_S
SINE_PARAMS = SINE_PARAMS_S

# == Definitions == #

WORK_POINT = Points.DroneBase.add(Point(x=600, y=-50, z=-200))


# == Implementation == #


async def main():
    generator = SineTrajectoryGenerator(LIMITS, SINE_PARAMS)

    async with Orchestrator() as o:
        await init(o)
        await run_experiment(o, generator)
        await finalise(o)


async def init(o: Orchestrator) -> None:
    with Status("Initialising free-flight experiment") as status:
        await o.execute(
            [
                Reset(),
                Device(DEVICE, DRONE_NO_ACTUATION),
                Robot(SetConfig(Config())),
                Robot(SetProfile(Profiles.Fast)),
                Robot(ReturnHome()),
                Robot(WaitSettled()),
                Robot(SetToolOffset(Offsets.Drone)),
                Robot(SetBlending(Blends.Small)),
                Robot(Move(MotionLinear(Points.DroneBase.add(Point(x=300))))),
                Robot(SetBlending(Blends.Zero)),
                Robot(Move(MotionLinear(WORK_POINT))),
                Robot(WaitSettled()),
                Robot(SetBlending(Blends.Small)),
                Robot(SetProfile(Profiles.Fast)),
                Robot(ResetMoveId()),
            ]
        )

        status.update("Biasing")
        await o.execute(
            [
                Sleep(1),
                Load(SetBias()),
                Device(DEVICE, [THROTTLE, WING_L, WING_R, 0.0, 0.0]),
            ]
        )

        if WIND_SPEED > 0.0:
            status.update("Settling wind speed")
            await o.execute(
                [
                    Wind(SetPowered(True)),
                    Wind(SetFanSpeed(WIND_SPEED)),
                    Wind(WaitSettled()),
                ]
            )


async def finalise(o: Orchestrator) -> None:
    await o.execute(
        [
            Robot(SetBlending(Blends.Small)),
            Robot(Move(MotionLinear(Points.DroneBase.add(Point(x=300))))),
            Robot(ReturnHome()),
            Robot(WaitSettled()),
        ]
    )


async def run_experiment(o: Orchestrator, generator: BaseTrajectoryGenerator) -> None:

    with ExperimentTui() as ui:
        ui.update(
            title="Free-Flight",
            total_progress=None,
            message="Generating trajectory",
        )

        all_points = []
        instructions = []

        for i in range(DURATION_S):
            timesteps = [i + k / POINT_FREQUENCY for k in range(POINT_FREQUENCY)]
            points = [generator.get_trajectory_point(t) for t in timesteps]

            for point in points:
                profile = Profile(
                    limit=ProfileLimit(
                        translation=point.velocity_t,
                        rotation=rad_to_deg(point.velocity_r),
                    )
                )

                if PLOT:
                    all_points.append(point.pose)

                point = WORK_POINT.add(point.pose)

                instructions.append(Robot(SetProfile(profile)))
                instructions.append(Robot(Move(MotionLinear(point))))
                instructions.append(Sleep(1 / POINT_FREQUENCY))

            instructions.append(Robot(WaitProgress((i + 1) * POINT_FREQUENCY - 5)))

        # Allow the robot to settle at the final point
        instructions.insert(-3, Robot(SetBlending(Blends.Zero)))
        instructions.insert(-3, Robot(SetProfile(Profiles.Fast)))

        # Wait until the robot completes the trajectory
        instructions.append(Robot(WaitSettled()))

        if not PLOT:
            ui.update(
                message="Executing trajectory",
                status=ExperimentStatus.RUNNING,
                total_progress=DURATION_S,
            )

            task = get_event_loop().create_task(status_task(o, ui))

            experiment_name = f"free-flight_w{WIND_SPEED}t{WING_L}u{WING_R}v{THROTTLE}"
            await o.new_experiment(experiment_name)

            await o.record(instructions)

            await o.execute(
                [
                    Wind(SetFanSpeed()),
                    Wind(SetPowered(False)),
                    Device(DEVICE, DRONE_NO_ACTUATION),
                    Robot(SetBlending(Blends.Zero)),
                    Robot(SetProfile(Profiles.Fast)),
                    Robot(Move(MotionLinear(WORK_POINT))),
                ]
            )

            ui.update(message="Saving experiment", status=ExperimentStatus.COMPLETE)
            await o.save_experiment()

            task.cancel()

    if PLOT:
        plot_trajectory(all_points)


async def status_task(o: Orchestrator, ui: ExperimentTui) -> None:
    while True:
        progress = await o.progress()

        ui.update(
            message="Executing trajectory",
            status=ExperimentStatus.RUNNING,
            current_progress=progress,
            total_progress=DURATION_S,
        )

        await sleep(1)


def plot_trajectory(points: list[Point]) -> None:
    import matplotlib.pyplot as plt

    x = [p.x for p in points]
    y = [p.y for p in points]
    z = [p.z for p in points]
    rx = [p.rx for p in points]
    ry = [p.ry for p in points]
    rz = [p.rz for p in points]

    fig, (ax1, ax2) = plt.subplots(
        1,
        2,
        figsize=(16, 7),
        subplot_kw={"projection": "3d"},
    )

    fig.suptitle("Robot Trajectory Visualization", fontsize=16)

    ax1.plot(x, y, z)

    ax1.set_title("Position Trajectory (meters)")
    ax1.set_xlabel("X axis")
    ax1.set_ylabel("Y axis")
    ax1.set_zlabel("Z axis")

    ax2.plot(rx, ry, rz)

    ax2.set_title("Orientation Trajectory (radians)")
    ax2.set_xlabel("RX axis (Pitch)")
    ax2.set_ylabel("RY axis (Roll)")
    ax2.set_zlabel("RZ axis (Yaw)")

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    plt.show()


if __name__ == "__main__":
    run(main())
