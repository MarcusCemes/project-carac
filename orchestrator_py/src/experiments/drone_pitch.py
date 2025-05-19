from dataclasses import replace
from time import sleep

from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *


BASE_POSITION = Point(x=600.0, y=50.0, z=400.0)
WIND_SPEED = 46


SLOW = Profile(
    acceleration_scale=25,
    deceleration_scale=25,
    translation_limit=500,
    rotation_limit=90,
)

FAST = Profile(
    acceleration_scale=80,
    deceleration_scale=80,
    translation_limit=1000,
    rotation_limit=360,
)


def main():
    with Orchestrator() as o:
        print(f"Status: {o.status()}")

        o.execute(
            [
                Reset(),
                LoadCell(SetBias()),
                RobotArm(SetProfile(FAST)),
                WindShape(RequestControl()),
                WindShape(EnablePower(True)),
                RobotArm(GoHome()),
                RobotArm(WaitSettled()),
                RobotArm(SetOffset(Point(z=300))),
            ]
        )

        o.new_experiment("drone_pitch")

        prepare(o)
        record_motion(o)
        return_to_base(o)

        o.execute(
            [
                WindShape(SetWindSpeed(WIND_SPEED)),
            ]
        )

        prepare(o)
        sleep(2)

        record_motion(o)

        o.execute([WindShape(SetWindSpeed(0))])
        return_to_base(o)

        o.save_experiment()

        o.execute(
            [
                RobotArm(SetProfile(FAST)),
                RobotArm(GoHome()),
                WindShape(EnablePower(False)),
                WindShape(ReleaseControl()),
            ]
        )


def prepare(o: Orchestrator):
    o.execute(
        [
            RobotArm(SetProfile(SLOW)),
            RobotArm(Move(MotionDirect(replace(BASE_POSITION, rx=90)))),
            RobotArm(WaitSettled()),
            Sleep(1.0),
        ]
    )


def record_motion(o: Orchestrator):
    o.record(
        [
            RobotArm(SetProfile(FAST)),
            RobotArm(Move(MotionLinear(replace(BASE_POSITION, rx=-90)))),
            RobotArm(WaitSettled()),
        ]
    )


def return_to_base(o: Orchestrator):
    o.execute(
        [
            RobotArm(SetProfile(SLOW)),
            RobotArm(Move(MotionDirect(replace(BASE_POSITION)))),
            RobotArm(WaitSettled()),
        ]
    )


if __name__ == "__main__":
    main()
