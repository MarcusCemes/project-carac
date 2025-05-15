from .orchestrator import Orchestrator
from .instructions import *

with Orchestrator() as o:
    print(f"Status: {o.status()}")

    point = Point()

    print(
        o.execute(
            [
                Sleep(0.5),
                RobotArm(Move(MotionLinear(point))),
                RobotArm(SetOffset(point)),
                RobotArm(SetProfile(Profile(velocity_scale=50))),
                RobotArm(WaitSettled()),
                LoadCell(SetBias()),
                LoadCell(SetStreaming(True)),
                WindShape(EnablePower(True)),
                WindShape(ReleaseControl()),
                WindShape(RequestControl()),
                WindShape(SetWindSpeed(50)),
            ]
        )
    )
