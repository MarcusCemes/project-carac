from sys import argv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scienceplots

from pipeline.dataframe import Columns

from .common import find_experiment
from .defs import *
from .lib.export import save_figure_tikz

del scienceplots


ANGLE_CUTOFF = 1.5
EXPERIMENT = "plunge"
NAME = "fast_pitch"
MODEL_COLOUR = "#882255"

INPUTS = [
    ("0.5 rad/s", 9, "#88ccee"),
    ("1 rad/s", 11, "#4477aa"),
    ("3 rad/s", 13, "#004488"),
    ("5 rad/s", 15, "#001144"),
]

opts: PlotOpts = {
    "x": Columns.AeroAngles[0],
    "xlabel": "Angle of Attack α [°]",
}


def main():
    dfs = load_dataframes()

    _, axs = plt.subplots(1, 2)

    axs[0].set_ylabel("Lift Force (N)")
    axs[1].set_ylabel("Drag Force (N)")

    for df, _, _ in dfs:
        df[Columns.AeroAngles] = np.rad2deg(df[Columns.AeroAngles].to_numpy())

    dfs[0][0].plot(
        **opts,
        y=Columns.AeroForcesModel[2],
        label="0.5 rad/s (model)",
        color=MODEL_COLOUR,
        ax=axs[0],
    )

    dfs[0][0].plot(
        **opts,
        y=Columns.AeroForcesModel[0],
        label="0.5 rad/s (model)",
        color=MODEL_COLOUR,
        ax=axs[1],
    )

    for df, label, colour in dfs:
        df.plot(**opts, y=Columns.AeroForces[2], label=label, color=colour, ax=axs[0])
        df.plot(**opts, y=Columns.AeroForces[0], label=label, color=colour, ax=axs[1])

    # plt.title("Plunge Aerodynamic Forces")
    plt.tight_layout()

    if "--save" in argv:
        plt.savefig(PLOT_PATH / f"{NAME}.png", dpi=DPI_IMAGE)

    if "--tikz" in argv:
        with plt.style.context(["science"]):
            fig = plt.figure(figsize=TIKZ_SIZE)
            ax = fig.gca()
            ax.set_ylim(-2.5, 2.5)

            dfs[0][0].plot(
                **opts,
                y=Columns.AeroForcesModel[2],
                label="Lift",
                color=MODEL_COLOUR,
                ax=ax,
            )

            for df, label, colour in dfs:
                df.plot(
                    **opts,
                    y=Columns.AeroForces[2],
                    label=label,
                    color=colour,
                    ax=ax,
                )

            save_figure_tikz((PLOT_PATH / f"{NAME}_lift"))
            plt.close(fig)

            fig = plt.figure(figsize=TIKZ_SIZE)
            ax = fig.gca()
            ax.set_ylim(0.0, 3.0)

            dfs[0][0].plot(
                **opts,
                y=Columns.AeroForcesModel[0],
                label="Drag",
                color=MODEL_COLOUR,
                ax=ax,
            )

            for df, label, colour in dfs:
                df.plot(
                    **opts,
                    y=Columns.AeroForces[0],
                    label=label,
                    color=colour,
                    ax=ax,
                )

            save_figure_tikz((PLOT_PATH / f"{NAME}_drag"))
            plt.close(fig)

    if not "--save" in argv and not "--tikz" in argv:
        plt.show()


def load_dataframes():
    return [(load_experiment(id), label, colour) for (label, id, colour) in INPUTS]


def load_experiment(id: int) -> pd.DataFrame:
    df = pd.read_parquet(find_experiment(EXPERIMENT, id))
    return trim_dataframe_by_threshold(df, "alpha", ANGLE_CUTOFF)


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
