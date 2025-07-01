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
from .lib.export import save_figure_tikz, save_legend


@dataclass
class Experiment:
    name: str
    files: tuple[Path, Path]
    x_axis: str
    x_label: str
    y_axis: str | None = None
    y_label: str | None = None


TITLE = "Lift & Drag Aerodynamic Forces"
LABELS = ("0.1 rad/s", "2 rad/s")

PITCH_EXPERIMENT = Experiment(
    "pitch",
    (find_experiment("axis-uncoupled", 16), find_experiment("axis-uncoupled", 17)),
    Columns.AeroAngles[0],  # 'alpha'
    "Angle of Attack α",
)

YAW_EXPERIMENT = Experiment(
    "yaw",
    (find_experiment("axis-uncoupled", 12), find_experiment("axis-uncoupled", 13)),
    Columns.AeroAngles[1],  # 'beta'
    "Angle of Sideslip β",
)

ROLL_EXPERIMENT = Experiment(
    "roll",
    (find_experiment("axis-uncoupled", 14), find_experiment("axis-uncoupled", 15)),
    Columns.WorldRotation[0],  # 'roll'
    "Roll Angle φ",
)


LEGENDS = True
SAVE_LEGEND = False
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
    if "grid" in argv:
        plot_grid()
        return

    for experiment in (
        PITCH_EXPERIMENT,
        YAW_EXPERIMENT,
        ROLL_EXPERIMENT,
    ):
        [df1, df2] = map(pd.read_parquet, experiment.files)

        # Remove some annoying outliers
        if experiment.x_axis == Columns.AeroAngles[1]:  # "beta"
            df2 = df2[df2[Columns.AeroAngles[1]] <= 0.2616].copy()

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle(TITLE, fontsize=TITLE_SIZE)

        y_axis_col_1 = experiment.y_axis or Columns.AeroForces[2]  # lift
        y_axis_col_2 = experiment.y_axis or Columns.AeroForces[0]  # drag

        y_axis_col_1_model = f"{y_axis_col_1}_model"
        y_axis_col_2_model = f"{y_axis_col_2}_model"

        plot_aero_force(
            df1,
            LABELS[0],
            COLOURS_1,
            experiment.x_axis,
            y_axis_col=y_axis_col_1,
            y_model_col=y_axis_col_1_model,
            ax=ax1,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=y_axis_col_1,
            y_model_col=y_axis_col_1_model,
            ax=ax1,
        )

        plot_aero_force(
            df1,
            LABELS[0],
            COLOURS_1,
            experiment.x_axis,
            y_axis_col=y_axis_col_2,
            y_model_col=y_axis_col_2_model,
            ax=ax2,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=y_axis_col_2,
            y_model_col=y_axis_col_2_model,
            ax=ax2,
        )

        ax1.set_xlabel(f"{experiment.x_label} [°]")
        ax1.set_ylabel(experiment.y_label or "Lift [N]")
        ax1.grid(True, linestyle="--", alpha=0.6)

        ax2.set_xlabel(f"{experiment.x_label} [°]")
        ax2.set_ylabel(experiment.y_label or "Drag [N]")
        ax2.grid(True, linestyle="--", alpha=0.6)

        if LEGENDS:
            ax1.legend(markerscale=3)
            ax2.legend(markerscale=3)

        plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

        name = f"aero_{experiment.name}"
        PLOT_PATH.mkdir(parents=True, exist_ok=True)

        if "--save" in argv:
            plt.savefig((PLOT_PATH / name).with_suffix(".png"), dpi=DPI_IMAGE)

        if "--tikz" in argv:
            is_sideslip = experiment.x_axis == Columns.AeroAngles[1]  # "beta"

            fig_tikz_lift = plt.figure(figsize=EXPORT_SIZE)
            ax_tikz_lift = fig_tikz_lift.gca()
            ax_tikz_lift.set_ylim(-2.5, 2.5)

            plot_aero_force(
                df1,
                LABELS[0],
                COLOURS_1,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[2],
                y_model_col=Columns.AeroForcesModel[2],
                ax=ax_tikz_lift,
            )

            plot_aero_force(
                df2,
                LABELS[1],
                COLOURS_2,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[2],
                y_model_col=Columns.AeroForcesModel[2],
                ax=ax_tikz_lift,
            )

            save_figure_tikz((PLOT_PATH / f"{name}_lift"))
            plt.close(fig_tikz_lift)

            fig_tikz_drag = plt.figure(figsize=EXPORT_SIZE)
            ax_tikz_drag = fig_tikz_drag.gca()
            ax_tikz_drag.set_ylim(0.0, 2.5)

            plot_aero_force(
                df1,
                LABELS[0],
                COLOURS_1,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[0],
                y_model_col=Columns.AeroForcesModel[0],
                legend=is_sideslip,
                ax=ax_tikz_drag,
            )

            plot_aero_force(
                df2,
                LABELS[1],
                COLOURS_2,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[0],
                y_model_col=Columns.AeroForcesModel[0],
                legend=is_sideslip,
                ax=ax_tikz_drag,
            )

            if is_sideslip:
                ax_tikz_drag.legend(markerscale=3)

            save_figure_tikz((PLOT_PATH / f"{name}_drag"))
            plt.close(fig_tikz_drag)

    if not "--save" in argv and not "--tikz" in argv:
        plt.show()


def plot_grid():

    y_axis_col_1 = Columns.AeroForces[2]  # lift
    y_axis_col_2 = Columns.AeroForces[0]  # drag

    y_axis_col_1_model = f"{y_axis_col_1}_model"
    y_axis_col_2_model = f"{y_axis_col_2}_model"

    _, axs = plt.subplots(1, 2, figsize=(12, 6))

    [df1, df2] = map(pd.read_parquet, PITCH_EXPERIMENT.files)

    ax1, ax2 = axs

    plot_aero_force(
        df1,
        LABELS[0],
        COLOURS_1,
        PITCH_EXPERIMENT.x_axis,
        y_axis_col=y_axis_col_1,
        y_model_col=y_axis_col_1_model,
        ax=ax1,
    )

    plot_aero_force(
        df2,
        LABELS[1],
        COLOURS_2,
        PITCH_EXPERIMENT.x_axis,
        y_axis_col=y_axis_col_1,
        y_model_col=y_axis_col_1_model,
        ax=ax1,
    )

    plot_aero_force(
        df1,
        LABELS[0],
        COLOURS_1,
        PITCH_EXPERIMENT.x_axis,
        y_axis_col=y_axis_col_2,
        y_model_col=y_axis_col_2_model,
        ax=ax2,
    )

    plot_aero_force(
        df2,
        LABELS[1],
        COLOURS_2,
        PITCH_EXPERIMENT.x_axis,
        y_axis_col=y_axis_col_2,
        y_model_col=y_axis_col_2_model,
        ax=ax2,
    )

    ax1.set_ylabel(PITCH_EXPERIMENT.y_label or "Lift [N]")
    ax2.set_ylabel(PITCH_EXPERIMENT.y_label or "Drag [N]")

    ax1.set_xlabel(f"{PITCH_EXPERIMENT.x_label} [°]")
    ax2.set_xlabel(f"{PITCH_EXPERIMENT.x_label} [°]")

    # ax1.grid(True, linestyle="--", alpha=0.6)
    # ax2.grid(True, linestyle="--", alpha=0.6)

    ax1.set_ylim(-3.0, 3.0)
    ax2.set_ylim(0.0, 3.0)

    if SAVE_LEGEND:
        save_legend(
            ax1,
            PLOT_PATH / "aero_legend.svg",
            ncol=2,
            markerscale=3,
        )

    ax1.legend().remove()
    ax2.legend().remove()

    # plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

    name = "aero_attack.svg"
    PLOT_PATH.mkdir(parents=True, exist_ok=True)
    plt.savefig((PLOT_PATH / name), dpi=DPI_IMAGE, transparent=True)

    _, axs = plt.subplots(2, 2, figsize=(12, 6))

    for i, experiment in enumerate([YAW_EXPERIMENT, ROLL_EXPERIMENT]):
        [df1, df2] = map(pd.read_parquet, experiment.files)

        # Remove some annoying outliers
        if experiment.x_axis == Columns.AeroAngles[1]:  # "beta"
            df2 = df2[df2[Columns.AeroAngles[1]] <= 0.2616].copy()

        ax1 = axs[0, i]
        ax2 = axs[1, i]

        plot_aero_force(
            df1,
            LABELS[0],
            COLOURS_1,
            experiment.x_axis,
            y_axis_col=y_axis_col_1,
            y_model_col=y_axis_col_1_model,
            ax=ax1,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=y_axis_col_1,
            y_model_col=y_axis_col_1_model,
            ax=ax1,
        )

        plot_aero_force(
            df1,
            LABELS[0],
            COLOURS_1,
            experiment.x_axis,
            y_axis_col=y_axis_col_2,
            y_model_col=y_axis_col_2_model,
            ax=ax2,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=y_axis_col_2,
            y_model_col=y_axis_col_2_model,
            ax=ax2,
        )

        if i == 0:
            ax1.set_ylabel(experiment.y_label or "Lift [N]")
            ax2.set_ylabel(experiment.y_label or "Drag [N]")
        else:
            ax1.set_ylabel("")
            ax2.set_ylabel("")

        ax1.set_xlabel("")
        ax2.set_xlabel(f"{experiment.x_label} [°]")

        # ax1.grid(True, linestyle="--", alpha=0.6)
        # ax2.grid(True, linestyle="--", alpha=0.6)

        ax1.set_ylim(-0.4, 0.2)
        ax2.set_ylim(0.0, 0.35)

    for ax in axs.flat:
        ax.legend().remove()

    # plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

    name = "aero_side_roll.svg"
    PLOT_PATH.mkdir(parents=True, exist_ok=True)

    plt.savefig((PLOT_PATH / name), dpi=DPI_IMAGE, transparent=True)


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
    df[x_deg_col] = np.rad2deg(df[x_axis_col])

    mid_point = len(df) // 2
    df_f = df.iloc[:mid_point]
    df_b = df.iloc[mid_point:]

    df_f.plot(
        kind="scatter",
        x=x_deg_col,
        y=y_axis_col,
        color=colors["forward"],
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        label=f"{name} (PU)",
        ax=ax,
        legend=legend,
    )

    df_b.plot(
        kind="scatter",
        x=x_deg_col,
        y=y_axis_col,
        color=colors["back"],
        s=POINT_SIZE,
        alpha=POINT_ALPHA,
        label=f"{name} (PD)",
        ax=ax,
        legend=legend,
    )

    df_f_sorted = df_f.sort_values(by=x_deg_col)
    df_b_sorted = df_b.sort_values(by=x_deg_col)

    df_f_sorted.plot(
        kind="line",
        x=x_deg_col,
        y=y_model_col,
        color=colors["model_fwd"],
        linewidth=LINE_WIDTH,
        label=f"{name} (PU) Model",
        ax=ax,
        legend=legend,
    )

    df_b_sorted.plot(
        kind="line",
        x=x_deg_col,
        y=y_model_col,
        color=colors["model_back"],
        linewidth=LINE_WIDTH,
        label=f"{name} (PD) Model",
        ax=ax,
        legend=legend,
    )


if __name__ == "__main__":
    main()
