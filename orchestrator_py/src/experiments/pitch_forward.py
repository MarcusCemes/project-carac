from math import cos, sin

from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *
from orchestrator.helpers import *

# Test matrix
ROT_SPEEDS_RAD: list[float] = [0.5, 1.0, 3.0, 5.0]
WIND_SPEEDS: list[int] = [0, 46]
OFFSET: list[float] = [0.0, DRONE_HEIGHT]
N_RUNS: int = 3

# Constants
SWEEP_ANGLE: float = 200


def main():
    with Orchestrator() as o:

        o.execute(init())

        for wind_speed in WIND_SPEEDS:
            for rot_speed in ROT_SPEEDS_RAD:
                for offset in OFFSET:
                    run_experiment(o, wind_speed, rot_speed, offset)

        o.execute(finalise())


def init() -> list[Instruction]:
    return [
        Reset(),
        WindShape(SetWindSpeed(0)),
        WindShape(EnablePower(True)),
        RobotArm(SetProfile(SLOW_PROFILE)),
        RobotArm(GoHome()),
        RobotArm(WaitSettled()),
        RobotArm(SetBlending(Blending())),
        RobotArm(SetProfile(FAST_PROFILE)),
        RobotArm(SetConfig(WORKING_CONFIG)),
        RobotArm(SetOffset(LOAD_CELL_OFFSET)),
        RobotArm(Move(MotionDirect(WORKING_POINT))),
        RobotArm(WaitSettled()),
        RobotArm(SetReportInterval(0.01)),
    ]


def finalise() -> list[Instruction]:
    return [
        RobotArm(GoHome()),
        WindShape(SetWindSpeed(0)),
        WindShape(EnablePower(False)),
        RobotArm(WaitSettled()),
    ]


def run_experiment(o: Orchestrator, wind_speed: int, rot_speed: float, offset: float):
    global previous_wind_speed

    print(f"Experiment: wind={wind_speed}, speed={rot_speed}, offset={offset}")

    name = f"drone_pitch_w{wind_speed}_r{rot_speed}_o{int(offset)}"
    o.new_experiment(name)

    if offset > 0.0:
        (instruction_from, instructions_to) = point_rotation(WORKING_POINT)
    else:
        (instruction_from, instructions_to) = axis_rotation(WORKING_POINT)

    through_pose = WORKING_POINT.add(Point(z=-offset))
    profile = create_profile(rotation_limit=rad_to_deg(rot_speed))

    print("Biasing...")
    o.execute(
        [
            RobotArm(Move(MotionLinear(WORKING_POINT))),
            RobotArm(WaitSettled()),
            Sleep(1),
            LoadCell(SetBias()),
            RobotArm(Move(MotionLinear(through_pose))),
        ]
    )

    if wind_speed > 0:
        print("Stabilising wind...")
        o.execute(
            [
                WindShape(SetWindSpeed(wind_speed)),
                Sleep(5),
            ]
        )

    for _ in range(N_RUNS):
        print("Moving to start position...")
        o.execute(
            [
                RobotArm(WaitSettled()),
                instruction_from,
                RobotArm(WaitSettled()),
                RobotArm(SetProfile(profile)),
            ]
        )

        print("Recording...")
        o.record(
            [
                *instructions_to,
                RobotArm(WaitSettled()),
            ]
        )

        print("Complete.")
        o.execute(
            [
                RobotArm(SetProfile(FAST_PROFILE)),
                RobotArm(Move(MotionLinear(through_pose))),
            ]
        )

    if wind_speed > 0:
        print("Turning off wind...")
        o.execute(
            [
                WindShape(SetWindSpeed(0)),
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

    f = RobotArm(Move(MotionLinear(point.add(Point(y=b, z=a, rx=half_angle)))))

    t = [
        RobotArm(
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
    f = RobotArm(Move(MotionLinear(point.add(Point(rx=half_angle)))))

    t = [
        RobotArm(SetBlending(Blending(1, 1, 1))),
        RobotArm(Move(MotionLinear(point))),
        RobotArm(SetBlending(Blending())),
        RobotArm(Move(MotionLinear(point.add(Point(rx=-half_angle))))),
    ]

    return (f, t)


if __name__ == "__main__":
    main()
