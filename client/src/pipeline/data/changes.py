from pandas import DataFrame
from scipy.spatial.transform import Rotation as R

from ..data.utils import LoadedExperiment
from ..dataframe import ROT_ANGLE_SEQ, Columns


def patch_robot_orientation(df: DataFrame) -> None:
    """
    Corrects flawed robot orientation data by undoing the pre-processing
    done by the orchestrator, re-interpreting the angles as intrinsic
    and then transforming them to the drone's coordinate system.
    """
    robot_angles = df[Columns.WorldRotation].to_numpy()

    # Undo the incorrect component swap to recover the original angles
    robot_angles[:, [0, 1]] = robot_angles[:, [1, 0]]
    robot_angles[:, 0] = -robot_angles[:, 0]

    # Create a ground-truth rotation object from the original angles
    rot_in_robot_frame = R.from_euler("XYZ", robot_angles)

    # Define the transform from the robot frame to the world frame
    C = R.from_euler("z", -90, degrees=True)

    # Change of basis into the robot's frame, apply the rotation, and change back
    rot_in_drone_frame = C * rot_in_robot_frame * C.inv()

    corrected_world_angles = rot_in_drone_frame.as_euler(ROT_ANGLE_SEQ)
    df[Columns.WorldRotation] = corrected_world_angles


def correct_load_orientation(df: DataFrame) -> None:
    "Rotate the load vector by 90 degrees clockwise around the z-axis."
    fx = df["load/fx"]
    fy = df["load/fy"]

    df["load/fx"] = -fy
    df["load/fy"] = fx


def add_idle_actuation(df: DataFrame) -> None:
    """Adds a row of idle actuation to the DataFrame."""

    for col, value in zip(Columns.DroneActuators, [0.05, 0.0, 0.0, 0.0, 0.0]):
        df[col] = value


def actuate_from_filename(df: DataFrame, experiment: LoadedExperiment) -> None:
    """Extracts the actuator values from the experiment name and adds them to the DataFrame."""
    params = experiment.parsed().parameters

    df[Columns.DroneActuators[0]] = params["u"]
    df[Columns.DroneActuators[1]] = params["s"]
    df[Columns.DroneActuators[2]] = params["t"]


def undo_sweep_l_fix(df: DataFrame) -> None:
    """The drone relay would flip the sign of the left wing sweep actuator."""
    df[Columns.DroneActuators[1]] *= -1.0


def remap_throttle(df: DataFrame) -> None:
    """Remap the throttle actuator to a range of 0.05 to 1.0."""

    def remap(x: float) -> float:
        return min(1.0, max(0.05, ((x + 1) / 2) * 0.95 + 0.05))

    df[Columns.DroneActuators[0]] = df[Columns.DroneActuators[0]].apply(remap)


def remap_sweep(df: DataFrame) -> None:
    """The model expects a different sweep range (-1 extended, 0.5 folded)."""
    df[Columns.DroneActuators[1]] *= -1.0
    df[Columns.DroneActuators[1:3]] += 0.5

    # Some experiments have sweep values that are too high, so we clamp them.
    df[Columns.DroneActuators[1:3]] = df[Columns.DroneActuators[1:3]].clip(-1, 0.5)
