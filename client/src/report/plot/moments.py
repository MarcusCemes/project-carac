from dataclasses import dataclass
from sys import argv
from pathlib import Path

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from pipeline.dataframe import Columns

from .common import find_experiment
from .defs import *
from .lib.export import save_figure_tikz


@dataclass
class Experiment:
    name: str
    files: tuple[Path, Path]
    x_axis: str
    x_label: str
    y_axis: str
    y_label: str


TITLE = "Aero Moments"
LABELS = ("0.1 rad/s", "2 rad/s")

PITCH_MOMENT = Experiment(
    "pitch",
    (find_experiment("axis-uncoupled", 16), find_experiment("axis-uncoupled", 17)),
    # Columns.AeroAngles[0],  # 'alpha'
    "time",
    "Angle of Attack α",
    y_axis=Columns.BodyMoment[1],
    y_label="Pitch Moment [Nm]",
)

YAW_MOMENT = Experiment(
    "yaw",
    (find_experiment("axis-uncoupled", 12), find_experiment("axis-uncoupled", 13)),
    # Columns.AeroAngles[1],  # 'beta'
    "time",
    "Angle of Sideslip β",
    y_axis=Columns.BodyMoment[2],
    y_label="Yaw Moment [Nm]",
)

LEGENDS = True
TITLE_SIZE = 16
LINE_WIDTH = 1.5
POINT_ALPHA = 0.8
POINT_SIZE = 2


COLOURS_1 = {
    "forward": "#88ccee",
    "back": "#4477aa",
    "model_fwd": "#004488",
    "model_back": "#001144",
}

COLOURS_2 = {
    "forward": "#ffbbbb",
    "back": "#ee6677",
    "model_fwd": "#aa3377",
    "model_back": "#882255",
}

EXPORT_SIZE = (3, 3)


def main():
    _, axs = plt.subplots(1, 2, figsize=(12, 5))

    df = pd.read_parquet(PITCH_MOMENT.files[0])
    df.plot(y=Columns.BodyMomentModel, x="time")
    print(Columns.BodyMomentModel)

    for i, experiment in enumerate([PITCH_MOMENT, YAW_MOMENT]):
        [df1, df2] = map(pd.read_parquet, experiment.files)

        col = experiment.y_axis
        col_model = f"{col}_model"

        ax: Axes = axs[i]

        plot_aero_force(
            df1,
            LABELS[0],
            COLOURS_1,
            experiment.x_axis,
            y_axis_col=col,
            y_model_col=col_model,
            ax=ax,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=col,
            y_model_col=col_model,
            ax=ax,
        )

        ax.grid(True, linestyle="--", alpha=0.6)
        ax.set_ylim(-0.25, 0.25)

        ax.set_xlabel(experiment.x_label)
        ax.set_ylabel(experiment.y_label)

    for ax in axs.flat:
        ax.legend().remove()

    axs[1].legend(ncols=2, markerscale=3)

    # plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

    name = "moments.svg"
    PLOT_PATH.mkdir(parents=True, exist_ok=True)

    # plt.savefig((PLOT_PATH / name), dpi=DPI_IMAGE, transparent=True)
    plt.show()


def plot_aero_force(
    df: pd.DataFrame,
    name: str,
    colors: dict,
    x_axis_col: str,
    y_axis_col: str,
    y_model_col: str,
    *,
    legend: bool = LEGENDS,
    ax: Axes,
):
    x_deg_col = f"{x_axis_col}_deg"
    # df[x_deg_col] = np.rad2deg(df[x_axis_col])
    df[x_deg_col] = df[x_axis_col]

    mid_point = len(df) // 2
    df_f = df.iloc[:mid_point]
    df_b = df.iloc[mid_point:]

    print(x_deg_col, y_axis_col, y_model_col)

    # df.plot(
    #     kind="scatter",
    #     x=x_deg_col,
    #     y=y_axis_col,
    #     color=colors["forward"],
    #     s=POINT_SIZE,
    #     alpha=POINT_ALPHA,
    #     label=f"{name} (F)",
    #     ax=ax,
    #     legend=legend,
    # )

    # df.plot(
    #     kind="scatter",
    #     x=x_deg_col,
    #     y=y_axis_col,
    #     color=colors["back"],
    #     s=POINT_SIZE,
    #     alpha=POINT_ALPHA,
    #     label=f"{name} (B)",
    #     ax=ax,
    #     legend=legend,
    # )

    # df_f_sorted = df_f.sort_values(by=x_deg_col)
    # df_b_sorted = df_b.sort_values(by=x_deg_col)

    df.plot(
        kind="line",
        x=x_deg_col,
        y=y_model_col,
        color=colors["model_fwd"],
        linewidth=LINE_WIDTH,
        label=f"{name} (F) Model",
        ax=ax,
        legend=legend,
    )

    # df_b_sorted.plot(
    #     kind="line",
    #     x=x_deg_col,
    #     y=y_model_col,
    #     color=colors["model_back"],
    #     linewidth=LINE_WIDTH,
    #     label=f"{name} (B) Model",
    #     ax=ax,
    #     legend=legend,
    # )


if __name__ == "__main__":
    main()
