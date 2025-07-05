from asyncio import get_event_loop, run, sleep
from math import sqrt
from random import Random

from carac.defs import DRONE_NO_ACTUATION
from rich.status import Status

from carac.helpers import format_name, rad_to_deg
from carac.motion import BaseTrajectoryGenerator, SineTrajectoryGenerator
from carac.prelude import *


# == Parameters == #

DURATION_S: int = 30

WIND_SPEEDS: list[float] = [0.0, 0.2, 0.4, 0.6]
# WING_L: float = 0.0  # O/C: 1 -> -0.5
# WING_R: float = 0.0  # O/C: -1 -> 0.5
# THROTTLE: float = -1

ACTUATION_INTERVAL = 5.0
POINT_FREQUENCY = 10

WITH_DRONE = False
DEVICE = "drone"

LIMITS = {
    "x": (-100.0, 100.0),
    "y": (-100.0, 100.0),
    "z": (-100.0, 100.0),
    "rx": (-40.0, 40.0),
    "ry": (-20.0, 20.0),
    "rz": (-15.0, 15.0),
}

SEEDS = [42, 1337, 101, 9999]

SINE_PARAMS = [
    {
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
    },
    {
        "x": [
            {"amp": 120.0, "freq": 0.6, "phase": 0.4},
            {"amp": 80, "freq": 0.8, "phase": 0.2},
        ],
        "y": [
            {"amp": 120.0, "freq": 1.2, "phase": 0.05},
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
    },
    {
        "x": [
            {"amp": 120.0, "freq": 0.7, "phase": 1.2},
            {"amp": 50.0, "freq": 0.4, "phase": 2.3},
        ],
        "y": [
            {"amp": 120.0, "freq": 0.95, "phase": 1.3},
            {"amp": 50.0, "freq": sqrt(0.5), "phase": 0.7},
        ],
        "z": [
            {"amp": 120.0, "freq": 0.6, "phase": 0.8},
            {"amp": 50.0, "freq": 0.4, "phase": 0.3},
        ],
        "rx": [
            {"amp": 30.0, "freq": 0.3, "phase": 0.5},
            {"amp": 16.0, "freq": sqrt(0.6), "phase": 1.2},
        ],
        "ry": [
            {"amp": 25.0, "freq": 0.9, "phase": 0.4},
            {"amp": 11.0, "freq": 0.7, "phase": 0.6},
        ],
        "rz": [
            {"amp": 15.0, "freq": 0.2, "phase": 1.5},
            {"amp": 8.0, "freq": sqrt(1.1), "phase": 2.3},
        ],
    },
]


# == Definitions == #

# WORK_POINT = Points.DroneBase.add(Point(x=600, y=-50, z=-200))
WORK_POINT = Points.DroneBase.add(Point(x=600, y=-50, z=-250))


# == Implementation == #


async def main():
    async with Orchestrator() as o:
        await init(o)
        await execute(o)
        await finalise(o)


async def init(o: Orchestrator) -> None:
    with Status("Initialising free-flight experiment"):
        await o.execute(
            [
                Reset(),
                *([Device(DEVICE, DRONE_NO_ACTUATION)] if WITH_DRONE else []),
                Robot(SetConfig(Configs.Free)),
                Robot(SetProfile(Profiles.Fast)),
                Robot(ReturnHome()),
                Robot(WaitSettled()),
                Robot(SetToolOffset(Offsets.Drone)),
                Robot(SetBlending(Blends.Small)),
                Robot(Move(MotionLinear(Points.DroneBase.add(Point(x=300))))),
                Robot(SetBlending(Blends.Zero)),
                Robot(Move(MotionLinear(WORK_POINT))),
                Robot(WaitSettled()),
                Wind(SetPowered(True)),
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


async def execute(o: Orchestrator) -> None:
    with ExperimentTui() as ui:
        ui.update(
            title="Free-Flight",
            total_progress=None,
            message="Generating trajectory",
            status=ExperimentStatus.IDLE,
        )

        for wind in WIND_SPEEDS:
            for i in range(3):
                params = SINE_PARAMS[i]
                seed = SEEDS[i]
                generator = SineTrajectoryGenerator(LIMITS, params)

                ui.update(title=f"Free-Flight (p={i}, w={wind}, d={seed})")
                await run_parameters(o, generator, i, seed, wind, ui)


async def run_parameters(
    o: Orchestrator,
    generator: BaseTrajectoryGenerator,
    p: int,
    seed: int,
    wind: float,
    ui: ExperimentTui,
):
    ui.update(message="Computing trajectory")

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

    # Wait until the robot completes the trajectory
    instructions.append(Robot(WaitSettled()))

    ui.update(
        message="Executing trajectory",
        status=ExperimentStatus.RUNNING,
        total_progress=DURATION_S,
    )

    task = get_event_loop().create_task(status_task(o, ui))

    experiment_name = format_name("free-flight", {"p": p, "w": wind, "d": seed})
    await o.new_experiment(experiment_name)

    ui.update(message="Biasing")
    await o.execute(
        [
            Robot(SetBlending(Blends.Small)),
            Robot(SetProfile(Profiles.Fast)),
            Robot(WaitSettled()),
            Sleep(1),
            Load(SetBias()),
        ]
    )

    if wind > 0.0:
        ui.update(message="Settling wind")
        await o.execute(
            [
                Wind(SetFanSpeed(wind)),
                Wind(WaitSettled()),
            ]
        )

    await o.execute(
        [
            instructions[3],
            Robot(WaitSettled()),
            Robot(ResetMoveId()),
        ]
    )

    ui.update(message="Recording experiment", status=ExperimentStatus.RUNNING)
    await o.record(
        [
            *instructions,
            # Saving takes a while, so we can power down here
            Wind(SetFanSpeed()),
            *([Device(DEVICE, DRONE_NO_ACTUATION)] if WITH_DRONE else []),
            Robot(SetBlending(Blends.Zero)),
            Robot(Move(MotionLinear(WORK_POINT))),
        ]
    )

    ui.update(message="Saving experiment", status=ExperimentStatus.COMPLETE)
    await o.save_experiment()

    task.cancel()


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
                "amp": round(amplitudes[i] * rng.uniform(0.8, 1.0), 2),
                # Randomize frequency within a reasonable range
                "freq": (
                    round(rng.uniform(0.4, 0.9), 3)
                    # Higher frequencies for pitching
                    if axis != "rx"
                    else round(rng.uniform(0.4, 1.3), 3)
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
