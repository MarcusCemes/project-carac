from pandas import DataFrame

from ..data.utils import LoadedExperiment
from ..dataframe import Columns


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
