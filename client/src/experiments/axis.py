from asyncio import run
from enum import EnumDict
from itertools import product

from rich.console import Console
from rich.progress import Progress
from rich.status import Status

from carac.defs import DRONE_NO_ACTUATION
from carac.helpers import (
    Vec3,
    Vec4,
    euler_xyz_intrinsic_to_quat,
    quat_conjugate,
    quat_multiply,
    quat_to_angle_rad,
    rad_to_deg,
    deg_to_rad,
)
from carac.prelude import *


# == Parameters == #

WITH_DRONE = False
ROT_SPEEDS_RAD: list[float] = [0.0, 0.1, 1.0, 3.0, 5.0]
WIND_SPEEDS: list[float] = [0.0, 0.5]
SWEEPS: list[float] = [-1.0, 0.5]
THRUSTS: list[float] = [-1.0]
N_RUNS: int = 1


# == Useful constants == #

ANGLE_LIMITS: Vec3 = tuple(map(deg_to_rad, (50, 20, 15)))  # type: ignore
DEVICE_NAME: str = "drone"
ROBOT_FREQUENCY: float = 1 / 20


class WorkingPoints(EnumDict):
    A = (
        Point(x=1000, y=450, z=500),
        Config(shoulder=ConfigKind.RightyPositive, elbow=ConfigKind.LeftyNegative),
    )

    B = (
        Point(x=900, y=-300, z=700),
        Config(),
    )

    C = (
        Point(x=1200, y=-200, z=500),
        Config(),
    )


# == Definitions == #


@dataclass
class Parameters:
    speeds: Vec3
    wind: float
    sweep_l: float
    sweep_r: float
    throttle: float
    runs: int

    def name(self, base: str) -> str:
        r = "-".join(f"{x}" for x in self.speeds)
        s = (
            f"{self.sweep_l}-{self.sweep_r}"
            if self.sweep_l != self.sweep_r
            else f"{self.sweep_l}"
        )

        return f"{base}_r{r}_w{self.wind}_s{s}_t{self.throttle}"


async def main():
    console = Console()

    (point, config) = WorkingPoints.A

    async with Orchestrator() as o:
        await init(o, point, config)

        parameters = [*iter_parameters()]

        with Progress(console=console, speed_estimate_period=120) as progress:
            task = progress.add_task("Running experiment", total=len(parameters))

            for i, param in enumerate(parameters):
                number = f"[{i + 1}/{len(parameters)}]"
                await run_experiment(o, point, param, console, progress, number)
                progress.update(task, advance=1)

        await finalise(o)


def iter_parameters():
    for sweep_l, sweep_r in product(SWEEPS, repeat=2) if WITH_DRONE else [(0.0, 0.0)]:
        for thrust in THRUSTS if WITH_DRONE else [0]:
            for wind in WIND_SPEEDS:
                for speeds in product(ROT_SPEEDS_RAD, repeat=3):

                    # Take only decoupled axis parameters
                    if sum(1 if x != 0 else 0 for x in speeds) == 1:

                        yield Parameters(
                            speeds=speeds,  # type: ignore
                            wind=wind,
                            sweep_l=-sweep_l,
                            sweep_r=sweep_r,
                            throttle=thrust,
                            runs=N_RUNS,
                        )

    # # Add a few coupled axis parameters
    coupled_sweeps: list[float] = [-1.0, 0.0, 0.5]
    coupled_wind: list[float] = [0.0, 0.5]
    coupled_speeds: list[float] = [0.0, 2.0]

    for wind in coupled_wind:
        for speeds in product(coupled_speeds, repeat=3):

            # Take only coupled axis parameters
            if sum(1 if x != 0 else 0 for x in speeds) >= 2:

                for sweep_l, sweep_r in (
                    product(coupled_sweeps, repeat=2) if WITH_DRONE else [(0.0, 0.0)]
                ):
                    yield Parameters(
                        speeds=speeds,  # type: ignore
                        wind=wind,
                        sweep_l=sweep_l,
                        sweep_r=sweep_r,
                        throttle=-1.0,  # no thrust
                        runs=N_RUNS,
                    )


async def init(o: Orchestrator, point: Point, config: Config):
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
                Robot(SetConfig(config)),
                Robot(SetBlending(Blending(BlendingKind.Joint, 250, 250))),
                Robot(SetToolOffset(Point())),
                Robot(Move(MotionDirect(Points.Base.add(Point(x=100))))),
                Robot(SetToolOffset(Offsets.Drone)),
                Robot(Move(MotionDirect(point))),
                Robot(SetBlending(Blending())),
                Robot(WaitSettled()),
            ]
        )


async def finalise(o: Orchestrator):
    with Status("Finalising experiment"):
        await o.execute(
            [
                Wind(SetFanSpeed(0)),
                Wind(SetPowered(False)),
                Robot(SetBlending(Blends.Large)),
                Robot(SetProfile(Profiles.Medium)),
                Robot(SetToolOffset(Point())),
                Robot(Move(MotionDirect(Points.Base.add(Point(x=100))))),
                Robot(ReturnHome()),
                Robot(SetBlending(Blending())),
                Robot(WaitSettled()),
            ]
        )


async def run_experiment(
    o: Orchestrator,
    point: Point,
    parameters: Parameters,
    console: Console,
    progress: Progress,
    number: str,
):
    task = progress.add_task("Computing trajectory", total=None)

    name = (
        "coupled-axis"
        if sum(1 if x != 0 else 0 for x in parameters.speeds) >= 2
        else "decoupled-axis"
    )

    name = parameters.name(name)
    await o.new_experiment(name)

    speeds = parameters.speeds
    wind = parameters.wind

    (_, velocity, _, end) = generate_maneuver(speeds, ANGLE_LIMITS)
    (pitch, roll, yaw) = map(rad_to_deg, end)

    console.print(
        f"[bold green]Experiment {number}[/]\nW: {parameters.wind}  S: {parameters.speeds}  A: {list(map(round, [pitch, roll, yaw]))}  V: {velocity:.2f}  S: {parameters.sweep_l}-{parameters.sweep_r}  T: {parameters.throttle}\n"
    )

    progress.refresh()

    from_pose = point.add(Point(rx=-pitch, ry=-roll, rz=-yaw))
    to_pose = point.add(Point(rx=pitch, ry=roll, rz=yaw))

    profile = Profile(
        limit=ProfileLimit(rotation=rad_to_deg(velocity)),
    )

    progress.update(task, description="Moving to bias point")
    await o.execute(
        [
            Wind(SetFanSpeed()),
            Robot(SetProfile(Profiles.Fast)),
            Robot(Move(MotionDirect(point))),
        ]
    )

    progress.update(task, description="Stabilising")
    await o.execute(
        [
            Robot(WaitSettled()),
            Wind(WaitSettled()),
            Sleep(1),
        ]
    )

    progress.update(task, description="Setting bias")
    await o.execute(
        [
            Load(SetBias()),
            Wind(SetFanSpeed(wind)),
        ]
    )

    if WITH_DRONE:
        drone_params = [
            parameters.throttle,
            parameters.sweep_l,
            parameters.sweep_r,
            0.0,
            0.0,
        ]

        await o.execute([Device(DEVICE_NAME, drone_params)])

    await o.execute(
        [
            Robot(Move(MotionLinear(from_pose))),
            Wind(WaitSettled()),
        ]
    )

    for run in range(parameters.runs):
        prefix = f"[{run}/{parameters.runs}]"
        progress.update(task, description=f"{prefix} Moving to start pose")

        await o.execute(
            [
                Robot(WaitSettled()),
                Robot(SetProfile(profile)),
            ]
        )

        progress.update(task, description=f"{prefix} Recording")
        await o.record(
            [
                Robot(Move(MotionLinear(to_pose))),
                Robot(Move(MotionLinear(from_pose))),
                Robot(WaitSettled()),
            ]
        )

    progress.update(task, description=f"Experiment complete")
    await o.execute(
        [
            Robot(SetProfile(Profiles.Fast)),
            Robot(Move(MotionLinear(point))),
            Wind(SetFanSpeed(0)),
        ]
    )

    if WITH_DRONE:
        await o.execute([Device(DEVICE_NAME, DRONE_NO_ACTUATION)])

    await o.save_experiment()

    progress.remove_task(task)


def generate_maneuver(
    euler_speeds_rps: Vec3,  # Input [vx, vy, vz] in radians/sec
    euler_bounds_rad: Vec3,  # Input [bound_x, bound_y, bound_z] in radians
) -> tuple[
    Vec4,
    float,
    Vec3,
    Vec3,
]:  # Returns (Q_relative, robot_speed_rad_s, start_euler_rad, end_euler_rad)
    """
    Computes the relative rotation quaternion for the maneuver, the robot's angular speed,
    and the start/end Euler poses, with inputs in radians.
    Euler convention is 'xyz' intrinsic.
    """
    vx_rps, vy_rps, vz_rps = euler_speeds_rps
    bx_rad, by_rad, bz_rad = euler_bounds_rad

    # 1. Calculate t_max (time to hit first Euler angle bound)
    t_max_s = float("inf")
    any_speed_nonzero = False

    if abs(vx_rps) > 1e-9:
        t_max_s = min(t_max_s, bx_rad / abs(vx_rps))
        any_speed_nonzero = True
    if abs(vy_rps) > 1e-9:
        t_max_s = min(t_max_s, by_rad / abs(vy_rps))
        any_speed_nonzero = True
    if abs(vz_rps) > 1e-9:
        t_max_s = min(t_max_s, bz_rad / abs(vz_rps))
        any_speed_nonzero = True

    if not any_speed_nonzero:  # All input speeds are zero
        return ((0.0, 0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    if t_max_s < 1e-9:  # Effectively zero t_max implies no motion
        return ((0.0, 0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    # 2. Determine Start and End Euler Poses (now directly in radians)
    end_euler_rad_tuple = (vx_rps * t_max_s, vy_rps * t_max_s, vz_rps * t_max_s)
    start_euler_rad_tuple = (
        -end_euler_rad_tuple[0],
        -end_euler_rad_tuple[1],
        -end_euler_rad_tuple[2],
    )

    # 3. Convert Euler Poses to Quaternions (input is already in radians)
    q_start = euler_xyz_intrinsic_to_quat(start_euler_rad_tuple)
    q_end = euler_xyz_intrinsic_to_quat(end_euler_rad_tuple)

    # 4. Calculate Relative Rotation Quaternion: q_relative = q_end * conjugate(q_start)
    q_start_conj = quat_conjugate(q_start)
    q_relative = quat_multiply(q_end, q_start_conj)

    # 5. Determine Total Angle of Relative Rotation (in radians)
    total_rotation_angle_rad = quat_to_angle_rad(q_relative)

    # 6. Calculate Robot's Angular Speed for the maneuver
    maneuver_duration_s = 2 * t_max_s

    robot_angular_speed_rad_per_s = 0.0
    if maneuver_duration_s > 1e-9:
        robot_angular_speed_rad_per_s = total_rotation_angle_rad / maneuver_duration_s
    elif total_rotation_angle_rad > 1e-9:  # Non-zero rotation in effectively zero time
        robot_angular_speed_rad_per_s = float("inf")

    return (
        q_relative,
        robot_angular_speed_rad_per_s,
        start_euler_rad_tuple,
        end_euler_rad_tuple,
    )


if __name__ == "__main__":
    run(main())
