from sys import argv
import pandas as pd
import matplotlib.pyplot as plt

from pipeline.dataframe import Columns

from .defs import *


INPUTS = [
    ("0.5 rad/s", "output/plunge/0009_drone_pitch_w0.46_r0.5_o320.parquet"),
    ("1 rad/s", "output/plunge/0011_drone_pitch_w0.46_r1.0_o320.parquet"),
    ("3 rad/s", "output/plunge/0013_drone_pitch_w0.46_r3.0_o320.parquet"),
    ("5 rad/s", "output/plunge/0015_drone_pitch_w0.46_r5.0_o320.parquet"),
]

opts: PlotOpts = {
    "x": Columns.AeroAngles[0],
    "xlabel": "Alpha α (rad)",
}


def main():
    dfs = [(label, pd.read_parquet(DATA_PATH / p)) for (label, p) in INPUTS]

    _, axs = plt.subplots(1, 2)

    axs[0].set_ylabel("Lift Force (N)")
    axs[1].set_ylabel("Drag Force (N)")

    for label, df in dfs:
        df.plot(**opts, y=Columns.AeroForces[0], label=label, ax=axs[0])
        df.plot(**opts, y=Columns.AeroForces[2], label=label, ax=axs[1])

    plt.title("Plunge Aerodynamic Forces")
    plt.tight_layout()

    if "--save" in argv:
        plt.savefig(PLOT_PATH / "aero_forces_model.png", dpi=DPI_IMAGE)

    plt.show()


if __name__ == "__main__":
    main()
