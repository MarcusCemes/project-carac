from asyncio import run
from math import cos, sin

from rich.status import Status

from carac.defs import DRONE_DISTANCE_Z
from carac.helpers import (
    deg_to_rad,
    rad_to_deg,
)
from carac.prelude import *


# Test matrix
ROT_SPEEDS_RAD: list[float] = [0.5, 1.0, 3.0, 5.0]
WIND_SPEEDS: list[float] = [0.0, 0.46]
OFFSET: list[float] = [0.0, DRONE_DISTANCE_Z]
N_RUNS: int = 2

# Constants
SWEEP_ANGLE: float = 200
ROBOT_INTERVAL: float = 0.01

POINT = Points.Working.add(Point(x=-200))
Y_OFFSET = -20


async def main():
    async with Orchestrator() as o:
        await init(o)

        print("Starting experiments...")
        for wind_speed in WIND_SPEEDS:
            for rot_speed in ROT_SPEEDS_RAD:
                for offset in OFFSET:
                    await run_experiment(o, wind_speed, rot_speed, offset)

        await finalise(o)


async def init(o: Orchestrator) -> None:
    with Status("Initialising plunge experiment"):
        await o.execute(
            [
                Reset(),
                Wind(SetFanSpeed(0)),
                Wind(SetPowered(True)),
                Robot(SetProfile(Profiles.Slow)),
                Robot(ReturnHome()),
                Robot(WaitSettled()),
                Robot(SetBlending(Blends.Zero)),
                Robot(SetProfile(Profiles.Fast)),
                Robot(SetConfig(Configs.Working)),
                Robot(SetToolOffset(Offsets.LoadCell)),
                Robot(Move(MotionDirect(POINT))),
                Robot(WaitSettled()),
                Robot(SetInterval(ROBOT_INTERVAL)),
                Robot(SetReporting(True)),
            ]
        )


async def finalise(o: Orchestrator) -> None:
    with Status("Finalising experiment"):
        await o.execute(
            [
                Robot(ReturnHome()),
                Wind(SetFanSpeed()),
                Wind(SetPowered(False)),
                Robot(WaitSettled()),
            ]
        )


async def run_experiment(
    o: Orchestrator,
    wind_speed: float,
    rot_speed: float,
    offset: float,
):
    global previous_wind_speed

    print(f"Experiment: wind={wind_speed}, speed={rot_speed}, offset={offset}")

    name = f"drone_pitch_w{wind_speed}_r{rot_speed}_o{int(offset)}"
    await o.new_experiment(name)

    if offset > 0.0:
        (instruction_from, instructions_to) = point_rotation(POINT)
    else:
        (instruction_from, instructions_to) = axis_rotation(POINT)

    through_pose = POINT.add(Point(z=-offset))
    profile = Profile(limit=ProfileLimit(rotation=rad_to_deg(rot_speed)))

    print("Biasing...")
    await o.execute(
        [
            Robot(Move(MotionLinear(POINT))),
            Robot(WaitSettled()),
            Wind(WaitSettled()),
            Sleep(1),
            Load(SetBias()),
            Robot(Move(MotionLinear(through_pose))),
        ]
    )

    if wind_speed > 0:
        print("Stabilising wind...")
        await o.execute(
            [
                Wind(SetFanSpeed(wind_speed)),
                Wind(WaitSettled()),
            ]
        )

    for _ in range(N_RUNS):
        print("Moving to start position...")
        await o.execute(
            [
                Robot(WaitSettled()),
                instruction_from,
                Robot(WaitSettled()),
                Robot(SetProfile(profile)),
            ]
        )

        print("Recording...")
        await o.record(
            [
                *instructions_to,
                Robot(WaitSettled()),
                *([Wind(SetFanSpeed())] if wind_speed > 0 else []),
                Robot(SetProfile(Profiles.Fast)),
                Robot(Move(MotionLinear(through_pose))),
            ]
        )

        print("Complete.")

    await o.save_experiment()


def point_rotation(
    point: Point,
    offset: float = DRONE_DISTANCE_Z,
    angle: float = SWEEP_ANGLE,
):
    half_angle = angle / 2

    alpha = deg_to_rad(half_angle - 90)
    a = offset * sin(alpha)
    b = offset * cos(alpha)

    f = Robot(Move(MotionLinear(point.add(Point(y=Y_OFFSET + b, z=a, rx=half_angle)))))

    t = [
        Robot(
            Move(
                MotionCircular(
                    point.add(Point(y=Y_OFFSET, z=-offset)),
                    point.add(Point(y=Y_OFFSET - b, z=a, rx=-half_angle)),
                )
            )
        )
    ]

    return (f, t)


def axis_rotation(point: Point, angle: float = SWEEP_ANGLE):
    half_angle = angle / 2
    f = Robot(Move(MotionLinear(point.add(Point(rx=half_angle)))))

    t = [
        Robot(SetBlending(Blending(BlendingKind.Joint, 1, 1))),
        Robot(Move(MotionLinear(point))),
        Robot(SetBlending(Blending())),
        Robot(Move(MotionLinear(point.add(Point(rx=-half_angle))))),
    ]

    return (f, t)


if __name__ == "__main__":
    run(main())
