from asyncio import get_event_loop, run, sleep
from math import sqrt
from random import Random

from carac.defs import DRONE_NO_ACTUATION
from rich.status import Status

from carac.helpers import format_name, rad_to_deg
from carac.motion import BaseTrajectoryGenerator, SineTrajectoryGenerator
from carac.prelude import *


# == Parameters == #

DURATION_S: int = 300

WIND_SPEED: float = 0.5
# WING_L: float = 0.0  # O/C: 1 -> -0.5
# WING_R: float = 0.0  # O/C: -1 -> 0.5
# THROTTLE: float = -1

ACTUATION_INTERVAL = 5.0
POINT_FREQUENCY = 10
PLOT = False

WITH_DRONE = True
DEVICE = "drone"

LIMITS = {
    "x": (-50.0, 50.0),
    "y": (-50.0, 50.0),
    "z": (-50.0, 50.0),
    "rx": (-50.0, 50.0),
    "ry": (-20.0, 20.0),
    "rz": (-15.0, 15.0),
}

SINE_PARAMS_A = {
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

SINE_PARAMS_B = {
    "x": [
        {"amp": 120.0, "freq": 0.4, "phase": 0.4},
        {"amp": 80, "freq": 1.1, "phase": 0.2},
    ],
    "y": [
        {"amp": 120.0, "freq": 1.1, "phase": 0.05},
        {"amp": 80, "freq": sqrt(0.7), "phase": 0.5},
    ],
    "z": [
        {"amp": 120.0, "freq": 0.7, "phase": 0.3},
        {"amp": 80, "freq": 0.4, "phase": 0.1},
    ],
    "rx": [
        {"amp": 35, "freq": 0.2},
        {"amp": 20, "freq": 0.7, "phase": 0.6},
    ],
    "ry": [
        {"amp": 25, "freq": 0.5},
        {"amp": 15, "freq": 0.5, "phase": 0.7},
    ],
    "rz": [
        {"amp": 15, "freq": 0.5, "phase": 0.2},
        {"amp": 8, "freq": sqrt(0.8), "phase": 0.8},
    ],
}


SEED_V1 = 42
SEED_V2 = 1337
SEED_V3 = 101
SEED_V4 = 9999

SINE_PARAMS_STRUCTURE = {
    "x": [{"amp": 0, "freq": 0, "phase": 0}, {"amp": 0, "freq": 0, "phase": 0}],
    "y": [{"amp": 0, "freq": 0, "phase": 0}, {"amp": 0, "freq": 0, "phase": 0}],
    "z": [{"amp": 0, "freq": 0, "phase": 0}, {"amp": 0, "freq": 0, "phase": 0}],
    "rx": [{"amp": 0, "freq": 0, "phase": 0}, {"amp": 0, "freq": 0, "phase": 0}],
    "ry": [{"amp": 0, "freq": 0, "phase": 0}, {"amp": 0, "freq": 0, "phase": 0}],
    "rz": [{"amp": 0, "freq": 0, "phase": 0}, {"amp": 0, "freq": 0, "phase": 0}],
}


# == Definitions == #

WORK_POINT = Points.DroneBase.add(Point(x=600, y=-50, z=-200))


# == Implementation == #


async def main():
    seed = SEED_V3

    # params = generate_sine_variation(SINE_PARAMS_STRUCTURE, LIMITS, seed=seed)
    generator = SineTrajectoryGenerator(LIMITS, SINE_PARAMS_A)

    async with Orchestrator() as o:
        await init(o)
        await run_experiment(o, generator, seed)
        await finalise(o)


async def init(o: Orchestrator) -> None:
    with Status("Initialising free-flight experiment") as status:
        await o.execute(
            [
                Reset(),
                *([Device(DEVICE, DRONE_NO_ACTUATION)] if WITH_DRONE else []),
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
            ]
        )

        status.update("Biasing")
        await o.execute(
            [
                Sleep(1),
                Load(SetBias()),
                # Removed in favour of random actuation
                # Device(DEVICE, [THROTTLE, WING_L, WING_R, 0.0, 0.0]),
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
            Wind(SetPowered(False)),
            Robot(SetBlending(Blends.Small)),
            Robot(Move(MotionLinear(Points.DroneBase.add(Point(x=300))))),
            Robot(ReturnHome()),
            Robot(WaitSettled()),
        ]
    )


async def run_experiment(
    o: Orchestrator, generator: BaseTrajectoryGenerator, seed: int
) -> None:

    with ExperimentTui() as ui:
        ui.update(
            title="Free-Flight",
            total_progress=None,
            message="Generating trajectory",
        )

        all_points = []
        instructions = []

        random_actuation = Random(56)
        last_actuation = 0.0

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

                actuation = [
                    random_actuation.uniform(-1, 0.5),
                    random_actuation.uniform(-0.5, 1.0),
                    random_actuation.uniform(-1.0, 0.5),
                    0.0,
                    0.0,
                ]

                instructions.append(Robot(SetProfile(profile)))
                instructions.append(Robot(Move(MotionLinear(point))))
                instructions.append(Sleep(1 / POINT_FREQUENCY))

                if WITH_DRONE and last_actuation >= ACTUATION_INTERVAL:
                    instructions.append(Device(DEVICE, actuation))
                    last_actuation = 0.0

            instructions.append(Robot(WaitProgress((i + 1) * POINT_FREQUENCY - 10)))

            last_actuation += 1

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

            experiment_name = format_name("free-flight", {"w": WIND_SPEED, "d": seed})
            await o.new_experiment(experiment_name)

            await o.execute(
                [
                    instructions[3],
                    Robot(WaitSettled()),
                    Robot(ResetMoveId()),
                ]
            )

            await o.record(
                [
                    *instructions,
                    Wind(SetFanSpeed()),
                    *([Device(DEVICE, DRONE_NO_ACTUATION)] if WITH_DRONE else []),
                    Robot(SetBlending(Blends.Zero)),
                    Robot(Move(MotionLinear(WORK_POINT))),
                ]
            )

            ui.update(message="Saving experiment", status=ExperimentStatus.COMPLETE)
            await o.save_experiment()

            task.cancel()

    if PLOT:
        plot_trajectory(all_points)


async def status_task(o: Orchestrator, ui: ExperimentTui) -> None:
    total = DURATION_S * POINT_FREQUENCY

    while True:
        progress = await o.progress()

        ui.update(
            message="Executing trajectory",
            status=ExperimentStatus.RUNNING,
            current_progress=round(progress),
            total_progress=total,
        )

        await sleep(0.25)


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


def generate_sine_variation(
    structure: dict,
    limits: dict,
    *,  # Make seed a keyword-only argument
    seed: int | None = None,
) -> dict:
    """
    Generates a new set of sine parameters that respects the given limits.

    Args:
        structure: The base structure defining how many sine components each axis has.
        limits: A dictionary of (min, max) tuples for each axis.
        seed: An integer to seed the random number generator for repeatable results.
              If None, the result will be different each time.
    """
    # Create a dedicated random number generator instance from the seed.
    # This avoids interfering with the global random state.
    rng = Random(seed)

    new_params = {}
    for axis, components in structure.items():
        min_lim, max_lim = limits[axis]
        total_range = max_lim - min_lim

        # This is the total "budget" for amplitudes on this axis.
        amplitude_budget = total_range / 2.0

        num_components = len(components)

        # Distribute the budget among the components using the seeded RNG
        splits = sorted(
            [0.0] + [rng.random() for _ in range(num_components - 1)] + [1.0]
        )
        amplitudes = [
            (splits[i + 1] - splits[i]) * amplitude_budget
            for i in range(num_components)
        ]

        rng.shuffle(amplitudes)

        new_components = []
        for i in range(num_components):
            comp = {
                # Add a small jitter to the amplitude for variety
                "amp": round(amplitudes[i] * rng.uniform(0.85, 1.0), 2),
                # Randomize frequency within a reasonable range
                "freq": (
                    round(rng.uniform(0.2, 1.5), 3)
                    # Higher frequencies for pitching
                    if axis != "rx"
                    else round(rng.uniform(0.2, 3.0), 3)
                ),
            }

            # If the original structure had a phase, add a random one
            if "phase" in components[i]:
                comp["phase"] = round(rng.random(), 3)

            new_components.append(comp)

        new_params[axis] = new_components

    return new_params


if __name__ == "__main__":
    run(main())
