from dataclasses import dataclass
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from pipeline.dataframe import Columns

from .common import find_experiment
from .defs import *
from .lib.plotting import COLOURS_1, COLOURS_2, plot_aero_force


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
    Columns.AeroAngles[0],  # 'alpha'
    "Angle of Attack α",
    y_axis=Columns.BodyMoment[1],
    y_label="Pitch Moment [Nm]",
)

YAW_MOMENT = Experiment(
    "yaw",
    (find_experiment("axis-uncoupled", 12), find_experiment("axis-uncoupled", 13)),
    Columns.AeroAngles[1],  # 'beta'
    "Angle of Sideslip β",
    y_axis=Columns.BodyMoment[2],
    y_label="Yaw Moment [Nm]",
)

LEGENDS = True
TITLE_SIZE = 16

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
            legend=LEGENDS,
            plot_scatter=False,
        )

        plot_aero_force(
            df2,
            LABELS[1],
            COLOURS_2,
            experiment.x_axis,
            y_axis_col=col,
            y_model_col=col_model,
            ax=ax,
            legend=LEGENDS,
            plot_scatter=False,
        )

        ax.grid(True, linestyle="--", alpha=0.6)
        ax.set_ylim(-0.25, 0.25)

        ax.set_xlabel(experiment.x_label)
        ax.set_ylabel(experiment.y_label)

    for ax in axs.flat:
        ax.legend().remove()

    axs[1].legend(ncols=2, markerscale=3)

    PLOT_PATH.mkdir(parents=True, exist_ok=True)

    plt.show()


if __name__ == "__main__":
    main()
