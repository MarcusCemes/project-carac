from dataclasses import dataclass
from sys import argv
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

from pipeline.dataframe import Columns

from .common import find_experiment
from .defs import *
from .lib.export import save_figure_tikz
from .lib.plotting import (
    COLOURS_1,
    COLOURS_2,
    plot_aero_force,
)


@dataclass
class Experiment:
    name: str
    files: tuple[Path, Path]
    x_axis: str
    x_label: str
    y_axis: str | None = None
    y_label: str | None = None


TITLE = "Lift & Drag Aerodynamic Forces"
LABELS = ("0.1 rad/s", "2.0 rad/s")

# # Attack 10 degrees
# ROLL_EXPERIMENT = Experiment(
#     "roll",
#     (find_experiment("attack-rotations", 24), find_experiment("attack-rotations", 25)),
#     Columns.WorldRotation[0],  # 'roll'
#     "Roll Angle φ",
# )


# YAW_EXPERIMENT = Experiment(
#     "yaw",
#     (find_experiment("attack-rotations", 26), find_experiment("attack-rotations", 27)),
#     Columns.AeroAngles[1],  # 'beta'
#     "Angle of Sideslip β",
# )

# # Attach 30 degrees
ROLL_EXPERIMENT = Experiment(
    "roll",
    (find_experiment("attack-rotations", 28), find_experiment("attack-rotations", 29)),
    Columns.WorldRotation[0],  # 'roll'
    "Roll Angle φ",
)


YAW_EXPERIMENT = Experiment(
    "yaw",
    (find_experiment("attack-rotations", 30), find_experiment("attack-rotations", 31)),
    Columns.AeroAngles[1],  # 'beta'
    "Angle of Sideslip β",
)


LEGENDS = True
TITLE_SIZE = 16

EXPORT_SIZE = (5, 2)


def main():

    for experiment in (ROLL_EXPERIMENT, YAW_EXPERIMENT):
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
            legend=LEGENDS,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=y_axis_col_1,
            y_model_col=y_axis_col_1_model,
            ax=ax1,
            legend=LEGENDS,
        )

        plot_aero_force(
            df1,
            LABELS[0],
            COLOURS_1,
            experiment.x_axis,
            y_axis_col=y_axis_col_2,
            y_model_col=y_axis_col_2_model,
            ax=ax2,
            legend=LEGENDS,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=y_axis_col_2,
            y_model_col=y_axis_col_2_model,
            ax=ax2,
            legend=LEGENDS,
        )

        ax1.set_xlabel(f"{experiment.x_label} [°]")
        ax1.set_ylabel(experiment.y_label or "Lift [N]")
        ax1.grid(True, linestyle="--", alpha=0.6)

        ax2.set_xlabel(f"{experiment.x_label} [°]")
        ax2.set_ylabel(experiment.y_label or "Drag [N]")
        ax2.grid(True, linestyle="--", alpha=0.6)

        if LEGENDS:
            ax1.legend()
            ax2.legend()

        plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

        name = f"attack_{experiment.name}"
        PLOT_PATH.mkdir(parents=True, exist_ok=True)

        if "--save" in argv:
            plt.savefig((PLOT_PATH / name).with_suffix(".png"), dpi=DPI_IMAGE)

        if "--tikz" in argv:
            fig_tikz_lift = plt.figure(figsize=EXPORT_SIZE)
            ax_tikz_lift = fig_tikz_lift.gca()
            ax_tikz_lift.set_ylim(1, 2)

            plot_aero_force(
                df1,
                LABELS[0],
                COLOURS_1,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[2],
                y_model_col=Columns.AeroForcesModel[2],
                ax=ax_tikz_lift,
                legend=LEGENDS,
            )

            plot_aero_force(
                df2,
                LABELS[1],
                COLOURS_2,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[2],
                y_model_col=Columns.AeroForcesModel[2],
                ax=ax_tikz_lift,
                legend=LEGENDS,
            )

            save_figure_tikz((PLOT_PATH / f"{name}_lift"))
            plt.close(fig_tikz_lift)

            fig_tikz_drag = plt.figure(figsize=EXPORT_SIZE)
            ax_tikz_drag = fig_tikz_drag.gca()
            ax_tikz_drag.set_ylim(0.5, 1.5)

            plot_aero_force(
                df1,
                LABELS[0],
                COLOURS_1,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[0],
                y_model_col=Columns.AeroForcesModel[0],
                ax=ax_tikz_drag,
                legend=LEGENDS,
            )

            plot_aero_force(
                df2,
                LABELS[1],
                COLOURS_2,
                experiment.x_axis,
                y_axis_col=Columns.AeroForces[0],
                y_model_col=Columns.AeroForcesModel[0],
                ax=ax_tikz_drag,
                legend=LEGENDS,
            )

            save_figure_tikz((PLOT_PATH / f"{name}_drag"))

            plt.close(fig_tikz_drag)

    if "--tikz" not in argv and "--save" not in argv:
        plt.show()


if __name__ == "__main__":
    main()
