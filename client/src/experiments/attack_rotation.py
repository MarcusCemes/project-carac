from asyncio import run
from itertools import product

from rich.status import Status
from scipy.spatial.transform import Rotation

from carac.defs import DRONE_NO_ACTUATION
from carac.helpers import rad_to_deg, format_name
from carac.prelude import *


# == Parameters == #

WITH_DRONE: bool = True
DEVICE_NAME: str = "drone"

ROT_SPEEDS_RAD: list[float] = [0.0, 0.1, 2.0]
WIND_SPEEDS: list[float] = [0.0, 0.2, 0.4, 0.6]
PITCH_ANGLES: list[float] = [10, 30]


# == Useful constants == #

ROLL_LIMIT: float = 35.0
YAW_LIMIT: float = 20.0

ROBOT_FREQUENCY: float = 1 / 20

# WORK_POINT = Points.DroneBase.add(Point(x=400, y=-400, z=-150))
WORK_POINT = Points.DroneBase.add(Point(x=550, y=-200, z=-150))


# == Definitions == #


@dataclass
class Parameters:
    pitch: float
    roll: float
    yaw: float
    wind: float

    def name(self) -> str:
        return format_name(
            "attack-rotations",
            {
                "p": self.pitch,
                "r": self.roll,
                "y": self.yaw,
                "w": self.wind,
            },
        )


async def main():
    async with Orchestrator() as o:
        await init(o)

        parameters = [*iter_parameters()]

        with ExperimentTui(disable=True) as tui:
            tui.update(total_progress=len(parameters))

            for i, param in enumerate(parameters):
                await run_experiment(o, param)
                tui.update(current_progress=i + 1)

        await finalise(o)


def iter_parameters():
    for wind in WIND_SPEEDS:
        for pitch in PITCH_ANGLES:
            for speeds in product(ROT_SPEEDS_RAD, repeat=2):
                if sum((1 if s != 0 else 0 for s in speeds)) == 1:
                    yield Parameters(
                        pitch=pitch,
                        yaw=speeds[0],
                        roll=speeds[1],
                        wind=wind,
                    )


async def init(o: Orchestrator):
    with Status("Initialising coupled axis experiment") as status:
        await o.execute(
            [
                Reset(),
                Wind(SetPowered(True)),
                Robot(SetProfile(Profiles.Medium)),
                Robot(SetBlending(Blending())),
                Robot(ReturnHome()),
                Robot(WaitSettled()),
                Robot(SetInterval(ROBOT_FREQUENCY)),
                Robot(SetReporting(True)),
            ]
        )

        status.update("Moving to working point")
        await o.execute(
            [
                Robot(SetConfig(Configs.Free)),
                Robot(SetBlending(Blends.Medium)),
                Robot(SetToolOffset(Offsets.Drone)),
                Robot(SetBlending(Blends.Zero)),
                Robot(Move(MotionDirect(WORK_POINT))),
                Robot(WaitSettled()),
            ]
        )


async def finalise(o: Orchestrator):
    with Status("Finalising experiment"):
        await o.execute(
            [
                Wind(SetFanSpeed(0)),
                Wind(SetPowered(False)),
                Robot(SetBlending(Blends.Small)),
                Robot(Move(MotionLinear(Points.DroneBase.add(Point(x=300))))),
                Robot(ReturnHome()),
                Robot(WaitSettled()),
            ]
        )


async def run_experiment(o: Orchestrator, parameters: Parameters):
    name = parameters.name()
    await o.new_experiment(name)

    pitch = parameters.pitch

    if parameters.roll > 0.0:
        p1 = [pitch, -ROLL_LIMIT, 0.0]
        p2 = [pitch, ROLL_LIMIT, 0.0]
    else:
        p1 = [pitch, 0.0, -YAW_LIMIT]
        p2 = [pitch, 0.0, YAW_LIMIT]

    from_pose = WORK_POINT.add(Point(rx=p1[0], ry=p1[1], rz=p1[2]))
    to_pose = WORK_POINT.add(Point(rx=p2[0], ry=p2[1], rz=p2[2]))

    print(to_pose)

    profile = Profile(
        limit=ProfileLimit(rotation=rad_to_deg(max(parameters.roll, parameters.yaw)))
    )

    await o.execute(
        [
            Wind(SetFanSpeed()),
            Robot(SetProfile(Profiles.Fast)),
            Robot(Move(MotionDirect(WORK_POINT))),
            Robot(WaitSettled()),
            Wind(WaitSettled()),
            Sleep(1),
            Load(SetBias()),
            Wind(SetFanSpeed(parameters.wind)),
        ]
    )

    if WITH_DRONE:
        await o.execute([Device(DEVICE_NAME, DRONE_NO_ACTUATION)])

    await o.execute(
        [
            Robot(Move(MotionLinear(from_pose))),
            Robot(WaitSettled()),
            Robot(SetProfile(profile)),
        ]
    )

    if parameters.wind > 0.0:
        await o.execute(
            [
                Sleep(3),
                Wind(WaitSettled()),
            ]
        )

    await o.record(
        [
            Robot(Move(MotionLinear(to_pose))),
            Robot(Move(MotionLinear(from_pose))),
            Robot(WaitSettled()),
        ]
    )

    await o.execute(
        [
            Robot(SetProfile(Profiles.Fast)),
            Robot(Move(MotionLinear(WORK_POINT))),
            Wind(SetFanSpeed(0)),
        ]
    )

    await o.save_experiment()


def convert_ext_to_ins(
    pitch: float,
    roll: float,
    yaw: float,
) -> tuple[float, float, float]:
    """Maps extrinsic angles to intrinsic angles (degrees)."""

    rotation = Rotation.from_euler("xyz", [pitch, roll, yaw], degrees=True).as_euler(
        "XYZ", degrees=True
    )

    return tuple(map(float, rotation))  # type: ignore


if __name__ == "__main__":
    run(main())
