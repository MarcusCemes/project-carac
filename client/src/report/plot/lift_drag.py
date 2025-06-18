from dataclasses import dataclass
from sys import argv

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from pipeline.dataframe import Columns

from .defs import *
from .lib.export import save_figure_tikz


@dataclass
class Experiment:
    name: str
    files: tuple[Path, Path]
    x_axis: str
    x_label: str


TITLE = "Lift & Drag Aerodynamic Forces"
LABELS = ("0.1 rad/s", "2.0 rad/s")

PITCH_EXPERIMENT = Experiment(
    "pitch",
    (
        OUTPUT_PATH
        / "decoupled/0016_coupled_axis_r0.1-0.0-0.0_w0.5_s-1.0_t-1.0.parquet",
        OUTPUT_PATH
        / "decoupled/0017_coupled_axis_r2.0-0.0-0.0_w0.5_s-1.0_t-1.0.parquet",
    ),
    Columns.AeroAngles[0],
    "Angle of Attack α [°]",
)

YAW_EXPERIMENT = Experiment(
    "yaw",
    (
        OUTPUT_PATH
        / "decoupled/0012_coupled_axis_r0.0-0.0-0.1_w0.5_s-1.0_t-1.0.parquet",
        OUTPUT_PATH
        / "decoupled/0013_coupled_axis_r0.0-0.0-2.0_w0.5_s-1.0_t-1.0.parquet",
    ),
    Columns.AeroAngles[1],
    "Angle of Sideslip β [°]",
)


TITLE_SIZE = 16
LINE_WIDTH = 1.5
POINT_ALPHA = 0.8
POINT_SIZE = 2


COLORS_1 = {
    "forward": "#88ccee",  # Light Blue
    "back": "#4477aa",  # Medium Blue
    "model_fwd": "#004488",  # Strong Blue
    "model_back": "#001144",  # Darkest Blue
}

COLORS_2 = {
    "forward": "#ffbbbb",  # Light Red
    "back": "#ee6677",  # Medium Red
    "model_fwd": "#aa3377",  # Strong Magenta/Red
    "model_back": "#882255",  # Darkest Magenta/Red
}


def main():
    for experiment in (PITCH_EXPERIMENT, YAW_EXPERIMENT):
        [df1, df2] = map(pd.read_parquet, experiment.files)

        # Remove some annoying outliers
        if experiment.x_axis == Columns.AeroAngles[1]:
            df2 = df2[df2[Columns.AeroAngles[1]] <= 0.2616]

        fig, (ax1, ax2) = plt.subplots(1, 2)
        fig.suptitle(TITLE, fontsize=TITLE_SIZE)

        plot_lift(df1, LABELS[0], COLORS_1, experiment.x_axis, ax=ax1)
        plot_lift(df2, LABELS[1], COLORS_2, experiment.x_axis, ax=ax1)

        plot_drag(df1, LABELS[0], COLORS_1, experiment.x_axis, ax=ax2)
        plot_drag(df2, LABELS[1], COLORS_2, experiment.x_axis, ax=ax2)

        ax1.set_xlabel(f"{experiment.x_label} [°]")
        ax1.set_ylabel("Lift [N]")
        ax1.grid(True, linestyle="--", alpha=0.6)
        ax1.legend()

        ax2.set_xlabel(f"{experiment.x_label} [°]")
        ax2.set_ylabel("Drag [N]")
        ax2.grid(True, linestyle="--", alpha=0.6)
        ax2.legend()

        plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

        path = PLOT_PATH / f"lift_drag_{experiment.name}"

        if "--save" in argv:
            plt.savefig(path.with_suffix(".png"), dpi=DPI_IMAGE)

        if "--tikz" in argv:
            fig = plt.figure(figsize=TIKZ_SIZE)
            ax = fig.gca()

            plot_lift(df1, LABELS[0], COLORS_1, experiment.x_axis, ax=ax)
            plot_lift(df2, LABELS[1], COLORS_2, experiment.x_axis, ax=ax)

            save_figure_tikz(path.with_stem(f"lift_drag_{experiment.name}_lift"))

            fig.clear()
            ax = fig.gca()

            plot_drag(df1, LABELS[0], COLORS_1, experiment.x_axis, ax=ax)
            plot_drag(df2, LABELS[1], COLORS_2, experiment.x_axis, ax=ax)

            save_figure_tikz(path.with_stem(f"lift_drag_{experiment.name}_drag"))

            plt.close(fig)

    plt.show()


def plot_lift(df: pd.DataFrame, name: str, colors: dict, x_axis: str, *, ax: Axes):

    mid_point = len(df) // 2

    df_f = df.iloc[:mid_point]
    df_b = df.iloc[mid_point:]

    x_f = np.rad2deg(df_f[x_axis].to_numpy())
    x_b = np.rad2deg(df_b[x_axis].to_numpy())

    ax.scatter(
        x_f,
        df_f[Columns.AeroForces[2]],
        color=colors["forward"],
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        label=f"{name} (F)",
    )

    ax.scatter(
        x_b,
        df_b[Columns.AeroForces[2]],
        color=colors["back"],
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        label=f"{name} (B)",
    )

    ax.plot(
        x_f,
        df_f[Columns.AeroForcesModel[2]],
        color=colors["model_fwd"],
        linewidth=LINE_WIDTH,
        label=f"Model {name} (F)",
    )

    ax.plot(
        x_b,
        df_b[Columns.AeroForcesModel[2]],
        color=colors["model_back"],
        linewidth=LINE_WIDTH,
        label=f"Model {name} (B)",
    )


def plot_drag(df: pd.DataFrame, name: str, colors: dict, x_axis: str, *, ax: Axes):
    if ax is None:
        ax = plt.gca()

    mid_point = len(df) // 2

    df_f = df.iloc[:mid_point]
    df_b = df.iloc[mid_point:]

    x_f = np.rad2deg(df_f[x_axis].to_numpy())
    x_b = np.rad2deg(df_b[x_axis].to_numpy())

    ax.scatter(
        x_f,
        df_f[Columns.AeroForces[0]],
        color=colors["forward"],
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        label=f"{name} (F)",
    )

    ax.scatter(
        x_b,
        df_b[Columns.AeroForces[0]],
        color=colors["back"],
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        label=f"{name} (B)",
    )

    ax.plot(
        x_f,
        df_f[Columns.AeroForcesModel[0]],
        color=colors["model_fwd"],
        linewidth=LINE_WIDTH,
        label=f"Model {name} (B)",
    )

    ax.plot(
        x_b,
        df_b[Columns.AeroForcesModel[0]],
        color=colors["model_back"],
        linewidth=LINE_WIDTH,
        label=f"Model {name} (B)",
    )


if __name__ == "__main__":
    main()
