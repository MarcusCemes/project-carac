from sys import argv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pipeline.dataframe import Columns

from .common import find_experiment
from .defs import *


ANGLE_CUTOFF = 1.5

INPUTS = [
    ("0.5 rad/s", 9),
    ("1 rad/s", 11),
    ("3 rad/s", 13),
    ("5 rad/s", 15),
]

opts: PlotOpts = {
    "x": Columns.AeroAngles[0],
    "xlabel": "Alpha α [rad]",
}


def main():
    dfs = [
        (label, pd.read_parquet(find_experiment("plunge", id)))
        for (label, id) in INPUTS
    ]

    _, axs = plt.subplots(1, 2)

    axs[0].set_ylabel("Lift Force (N)")
    axs[1].set_ylabel("Drag Force (N)")

    for label, df in dfs:
        df = trim_dataframe_by_threshold(df, "alpha", ANGLE_CUTOFF)

        df.plot(**opts, y=Columns.AeroForces[2], label=label, ax=axs[0])
        df.plot(**opts, y=Columns.AeroForces[0], label=label, ax=axs[1])

    plt.title("Plunge Aerodynamic Forces")
    plt.tight_layout()

    if "--save" in argv:
        plt.savefig(PLOT_PATH / "aero_forces_model.png", dpi=DPI_IMAGE)

    plt.show()


def trim_dataframe_by_threshold(df, column, threshold):
    """
    Trim dataframe to range between two points where |column| crosses threshold,
    searching outward from midpoint.
    """
    values = np.abs(df[column].values)
    mid = len(values) // 2

    # Find where |values| > threshold
    crosses = values > threshold

    # Find first crossing going backward from midpoint
    left_crosses = crosses[:mid][::-1]  # Reverse first half
    left_idx = mid - np.argmax(left_crosses) - 1 if np.any(left_crosses) else 0

    # Find first crossing going forward from midpoint
    right_crosses = crosses[mid:]
    right_idx = (
        mid + np.argmax(right_crosses) if np.any(right_crosses) else len(values) - 1
    )

    return df.iloc[left_idx : right_idx + 1]


if __name__ == "__main__":
    main()
