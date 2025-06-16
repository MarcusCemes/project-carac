from asyncio import run

from rich.status import Status

from carac.prelude import *

POINT = Point(x=-100, y=-100, z=400)


async def main():
    async with Orchestrator() as o:
        with Status("Moving to working point") as status:

            await o.execute(
                [
                    Reset(),
                    Robot(SetProfile(Profiles.Slow)),
                    Robot(SetToolOffset(Point())),
                    Robot(SetConfig(Config())),
                    Robot(SetBlending(Blending())),
                    Robot(Move(MotionDirect(POINT))),
                ]
            )

            await o.new_experiment("axis_test")

            status.update("Recording axis movements")
            await o.record(
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

            await o.save_experiment()

            status.update("Returning to home position")
            await o.execute(
                [
                    Robot(ReturnHome()),
                    Robot(WaitSettled()),
                ]
            )


if __name__ == "__main__":
    run(main())
