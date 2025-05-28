from itertools import product

from orchestrator.orchestrator import Orchestrator
from orchestrator.instructions import *
from orchestrator.helpers import *

# Test matrix
ROT_SPEEDS_RAD: list[float] = [0.0, 0.5, 1.0]
WIND_SPEEDS: list[float] = [0, 0.46]
N_RUNS: int = 2

# Constants
ANGLE_LIMITS: Vec3 = tuple(map(deg_to_rad, (40, 20, 15)))  # type: ignore

REF_POINT = CLOSER_WORKING_POINT.add(Point(x=-100))


@dataclass
class Parameters:
    speeds: Vec3
    wind: float


def main():
    with Orchestrator() as o:

        print("Initializing orchestrator...")
        o.execute(init())

        print("Starting experiments...")
        for parameters in generate_parameters():
            run_experiment(o, parameters)

        o.execute([Sleep(2)])

        print("Finalising orchestrator...")
        o.execute(finalise())


def generate_parameters():
    for wind in WIND_SPEEDS:
        for pitch, roll, yaw in product(ROT_SPEEDS_RAD, repeat=3):
            if (pitch, roll, yaw) != (0.0, 0.0, 0.0):
                yield Parameters(speeds=(pitch, roll, yaw), wind=wind)


def init() -> list[Instruction]:
    return [
        Reset(),
        Wind(SetFanSpeed(0)),
        Wind(SetPowered(True)),
        Robot(SetProfile(SLOW_PROFILE)),
        Robot(SetBlending(Blending())),
        Robot(SetConfig(WORKING_CONFIG)),
        Robot(SetToolOffset(DRONE_OFFSET)),
        Robot(Move(MotionDirect(REF_POINT))),
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


def run_experiment(o: Orchestrator, parameters: Parameters):
    global previous_wind_speed

    print(f"Experiment: wind={parameters.wind}, speeds={parameters.speeds}")

    name = f"drone_pitch_r{"-".join(map(str, parameters.speeds))}_w{parameters.wind}"
    o.new_experiment(name)

    # (pitch, roll, yaw) = map(rad_to_deg, parameters.speeds)
    speeds = parameters.speeds
    wind = parameters.wind

    (_quat, velocity, start, _end) = generate_maneuver(speeds, ANGLE_LIMITS)
    (pitch, roll, yaw) = map(rad_to_deg, start)

    print(f"> Rotation angles: {round(pitch)} {round(roll)} {round(yaw)}")
    print(f"> Velocity: {velocity:.2f} rad/s")

    from_pose = REF_POINT.add(Point(rx=-pitch, ry=-roll, rz=-yaw))
    to_pose = REF_POINT.add(Point(rx=pitch, ry=roll, rz=yaw))

    profile = Profile(
        limit=ProfileLimit(rotation=rad_to_deg(velocity)),
        scale=ProfileScale(50, 50),  # REMOVE THIS FOR PROPER TEST
    )

    print("Biasing...")
    o.execute(
        [
            Robot(Move(MotionDirect(REF_POINT))),
            Robot(WaitSettled()),
            Sleep(1),
            Wind(WaitSettled()),
            Load(SetBias()),
            Wind(SetFanSpeed(wind)),
            Robot(Move(MotionLinear(from_pose))),
        ]
    )

    if wind > 0:
        print("Stabilising wind...")
        o.execute([Wind(WaitSettled())])

    for _ in range(N_RUNS):
        print("Moving to start position...")
        o.execute(
            [
                Robot(WaitSettled()),
                Robot(Move(MotionLinear(from_pose))),
                Robot(WaitSettled()),
                Robot(SetProfile(profile)),
            ]
        )

        print("Recording...")
        o.record(
            [
                Robot(Move(MotionLinear(to_pose))),
                Robot(WaitSettled()),
            ]
        )

        o.record(
            [
                Robot(Move(MotionLinear(from_pose))),
                Robot(WaitSettled()),
            ]
        )

    print("Complete.")
    o.execute(
        [
            Robot(SetProfile(FAST_PROFILE)),
            Robot(Move(MotionLinear(REF_POINT))),
            Wind(SetFanSpeed(0)),
        ]
    )

    o.save_experiment()


def generate_maneuver(
    euler_speeds_rps: Vec3,  # Input [vx, vy, vz] in radians/sec
    euler_bounds_rad: Vec3,  # Input [bound_x, bound_y, bound_z] in radians
) -> tuple[
    Vec4, float, Vec3, Vec3
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
    main()
