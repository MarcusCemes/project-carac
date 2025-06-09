from math import cos, sin

from rich.console import Console

from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *
from orchestrator.helpers import *

# Test matrix
ROT_SPEEDS_RAD: list[float] = [0.5, 1.0, 3.0, 5.0]
WIND_SPEEDS: list[float] = [0.0, 0.46]
OFFSET: list[float] = [0.0, DRONE_HEIGHT]
N_RUNS: int = 2

# Constants
SWEEP_ANGLE: float = 200

POINT = WORKING_POINT.add(Point(x=-200))


def main():
    console = Console()

    with Orchestrator() as o:

        with console.status("[bold green]Initialising..."):
            o.execute(init())

        print("Starting experiments...")
        for wind_speed in WIND_SPEEDS:
            for rot_speed in ROT_SPEEDS_RAD:
                for offset in OFFSET:
                    run_experiment(o, wind_speed, rot_speed, offset)

        with console.status("[bold green]Finalising..."):
            o.execute(finalise())


def init() -> list[Instruction]:
    return [
        Reset(),
        Wind(SetFanSpeed(0)),
        Wind(SetPowered(True)),
        Robot(SetProfile(SLOW_PROFILE)),
        Robot(ReturnHome()),
        Robot(WaitSettled()),
        Robot(SetBlending(Blending())),
        Robot(SetProfile(FAST_PROFILE)),
        Robot(SetConfig(WORKING_CONFIG)),
        Robot(SetToolOffset(LOAD_CELL_OFFSET)),
        Robot(Move(MotionDirect(POINT))),
        Robot(WaitSettled()),
        Robot(SetFrequency(0.01)),
        Robot(SetReporting(True)),
    ]


def finalise() -> list[Instruction]:
    return [
        Robot(ReturnHome()),
        Wind(SetFanSpeed(0)),
        Wind(SetPowered(False)),
        Robot(WaitSettled()),
    ]


def run_experiment(o: Orchestrator, wind_speed: float, rot_speed: float, offset: float):
    global previous_wind_speed

    print(f"Experiment: wind={wind_speed}, speed={rot_speed}, offset={offset}")

    name = f"drone_pitch_w{wind_speed}_r{rot_speed}_o{int(offset)}"
    o.new_experiment(name)

    if offset > 0.0:
        (instruction_from, instructions_to) = point_rotation(POINT)
    else:
        (instruction_from, instructions_to) = axis_rotation(POINT)

    through_pose = POINT.add(Point(z=-offset))
    profile = create_profile(rotation_limit=rad_to_deg(rot_speed))

    print("Biasing...")
    o.execute(
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
        o.execute(
            [
                Wind(SetFanSpeed(wind_speed)),
                Wind(WaitSettled()),
            ]
        )

    for _ in range(N_RUNS):
        print("Moving to start position...")
        o.execute(
            [
                Robot(WaitSettled()),
                instruction_from,
                Robot(WaitSettled()),
                Robot(SetProfile(profile)),
            ]
        )

        print("Recording...")
        o.record(
            [
                *instructions_to,
                Robot(WaitSettled()),
            ]
        )

        print("Complete.")
        o.execute(
            [
                Robot(SetProfile(FAST_PROFILE)),
                Robot(Move(MotionLinear(through_pose))),
            ]
        )

    if wind_speed > 0:
        print("Turning off wind...")
        o.execute(
            [
                Wind(SetFanSpeed(0)),
                Sleep(10),
            ]
        )

    o.save_experiment()


def point_rotation(
    point: Point,
    offset: float = DRONE_HEIGHT,
    angle: float = SWEEP_ANGLE,
):
    half_angle = angle / 2

    alpha = deg_to_rad(half_angle - 90)
    a = offset * sin(alpha)
    b = offset * cos(alpha)

    f = Robot(Move(MotionLinear(point.add(Point(y=b, z=a, rx=half_angle)))))

    t = [
        Robot(
            Move(
                MotionCircular(
                    point.add(Point(z=-offset)),
                    point.add(Point(y=-b, z=a, rx=-half_angle)),
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
    main()
