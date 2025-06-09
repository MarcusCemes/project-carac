from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *
from orchestrator.helpers import *

POINT = Point(x=-100, y=-100, z=400)


def main():
    with Orchestrator() as o:

        o.execute(
            [
                Reset(),
                Robot(SetProfile(SLOW_PROFILE)),
                Robot(SetToolOffset(Point())),
                Robot(SetConfig(Config())),
                Robot(SetBlending(Blending())),
                Robot(Move(MotionDirect(POINT))),
            ]
        )

        o.new_experiment("test_axis")

        o.record(
            [
                Robot(Move(MotionLinear(POINT.add(Point(y=200))))),
                Robot(WaitSettled()),
                Robot(Move(MotionLinear(POINT))),
                Robot(Move(MotionLinear(POINT.add(Point(x=-200))))),
                Robot(WaitSettled()),
                Robot(Move(MotionLinear(POINT))),
                Robot(Move(MotionLinear(POINT.add(Point(z=200))))),
                Robot(WaitSettled()),
                Robot(Move(MotionLinear(POINT))),
                Robot(WaitSettled()),
            ]
        )

        o.save_experiment()

        o.execute(
            [
                Robot(ReturnHome()),
                Robot(WaitSettled()),
            ]
        )


if __name__ == "__main__":
    main()
