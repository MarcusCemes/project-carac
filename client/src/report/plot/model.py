from dataclasses import asdict, dataclass
from sys import argv

import pandas as pd
import matplotlib.pyplot as plt

from pipeline.dataframe import Columns

from .common import find_experiment
from .defs import *


@dataclass
class Opts:
    x: str
    xlabel: str


# FILE = find_experiment("axis-uncoupled", 13)
# FILE = find_experiment("plunge", 14)
# FILE = find_experiment("attack-rotations", 28)
FILE = find_experiment("attack-rotations", 24)

OPTS: list[Opts] = [
    Opts(
        Columns.AeroAngles[0],
        # Columns.Time,
        "Alpha α [rad]",
    ),
    Opts(
        Columns.AeroAngles[1],
        # Columns.Time,
        "Beta β [rad]",
    ),
    Opts(
        Columns.Time,
        "Time [s]",
    ),
]


def main():
    df = pd.read_parquet(OUTPUT_PATH / FILE)

    plot_model(df)
    plot_inputs(df)

    plt.show()


def plot_model(df: pd.DataFrame):
    for opts in OPTS:
        _, axs = plt.subplots(1, 2)

        df.plot(**asdict(opts), y=Columns.AeroForcesModel, ax=axs[0])
        df.plot(**asdict(opts), y=Columns.BodyForceModel, ax=axs[1])

        plt.title(f"Aerodynamic Forces Model ({opts.xlabel})")
        plt.tight_layout()

        path = PLOT_PATH / f"model_{opts.x}.png"

        if "--save" in argv:
            plt.savefig(path, dpi=DPI_IMAGE)


def plot_inputs(df: pd.DataFrame):
    df = pd.read_parquet(OUTPUT_PATH / FILE)

    for opts in OPTS:
        args = asdict(opts)

        plt.figure()

        ax1 = plt.subplot2grid((3, 2), (0, 0))
        ax2 = plt.subplot2grid((3, 2), (0, 1))
        ax3 = plt.subplot2grid((3, 2), (1, 0))
        ax4 = plt.subplot2grid((3, 2), (1, 1))

        ax5 = plt.subplot2grid((3, 2), (2, 0), colspan=2)

        df.plot(**args, y=Columns.WorldRotation, ax=ax1)
        df.plot(**args, y=Columns.Attitude, ax=ax2)
        df.plot(**args, y=Columns.AeroAngularVelocity, ax=ax3)
        df.plot(**args, y=Columns.AeroVelocity, ax=ax4)
        df.plot(**args, y=Columns.DroneActuators, ax=ax5)

        plt.title(f"Model Inputs ({opts.xlabel})")

        plt.tight_layout()

        if "--save" in argv:
            plt.savefig(PLOT_PATH / f"model_inputs_{opts.x}.png", dpi=DPI_IMAGE)


if __name__ == "__main__":
    main()
