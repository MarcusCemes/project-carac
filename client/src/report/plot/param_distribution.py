from dataclasses import dataclass
from sys import argv

import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.axes import Axes
import scienceplots

from .defs import DPI_IMAGE, PLOT_PATH, TIKZ_SIZE
from .common import find_experiment

del scienceplots

FILE_1 = find_experiment("free-flight-3", 3)
FILE_2 = find_experiment("free-flight-3", 4)


NAME = "parameters"

VIOLIN_COLOR_1 = "#ee6677"
VIOLIN_COLOR_2 = "#4477aa"


@dataclass
class Parameter:
    col_name: str
    label: str


PARAMETERS_1 = [
    Parameter("alpha", r"$\alpha$"),
    Parameter("beta", r"$\beta$"),
    Parameter("p", r"$p$"),
    Parameter("q", r"$q$"),
    Parameter("r", r"$r$"),
]

PARAMETERS_2 = [
    Parameter("u", r"$u$"),
    Parameter("v", r"$v$"),
    Parameter("w", r"$w$"),
]


def main():
    df1 = pd.read_parquet(FILE_1)
    df2 = pd.read_parquet(FILE_2)

    with plt.style.context(["science", "ieee"]):
        plt.figure(figsize=TIKZ_SIZE)
        gs = GridSpec(2, 1, height_ratios=[2, 1])

        ax1 = plt.subplot(gs[0])
        ax2 = plt.subplot(gs[1])

        plot_violin(df1, df2, PARAMETERS_1, ax=ax1)
        plot_violin(df1, df2, PARAMETERS_2, ax=ax2)

        for ax in [ax1, ax2]:
            ax.set_xlabel("")
            # ax.tick_params(axis="y", length=0)
            # sns.despine(ax=ax, left=True, bottom=False)

        # plt.tight_layout()
        # plt.subplots_adjust(hspace=0.1)

        ax1.set_ylabel("[rad, rad/s]")
        ax2.set_ylabel("[m/s]")

        if "--tikz" in argv:
            PLOT_PATH.mkdir(parents=True, exist_ok=True)
            output_file = (PLOT_PATH / NAME).with_suffix(".eps")
            plt.savefig(output_file, dpi=DPI_IMAGE)

        else:
            plt.show()


def plot_violin(
    df1: pd.DataFrame,
    df2: pd.DataFrame,
    parameters: list[Parameter],
    *,
    ax: Axes,
):
    columns = [p.col_name for p in parameters]

    # Process first dataframe
    df1_melted = df1[columns].copy()
    df1_melted = df1_melted.melt(var_name="Parameter", value_name="Value")
    df1_melted["split_cat"] = "A"

    # Process second dataframe
    df2_melted = df2[columns].copy()
    df2_melted = df2_melted.melt(var_name="Parameter", value_name="Value")
    df2_melted["split_cat"] = "B"

    # Combine both dataframes
    df = pd.concat([df1_melted, df2_melted], ignore_index=True)

    y_order = [p.label for p in parameters]
    label_map = {p.col_name: p.label for p in parameters}

    df["Parameter"] = df["Parameter"].map(label_map)

    sns.violinplot(
        df,
        x="Value",
        y="Parameter",
        hue="split_cat",
        order=y_order,
        split=True,
        orient="h",
        inner=None,
        palette=[VIOLIN_COLOR_1, VIOLIN_COLOR_2],
        legend=False,
        linewidth=0,
        density_norm="width",
        cut=0,
        ax=ax,
    )


if __name__ == "__main__":
    main()
