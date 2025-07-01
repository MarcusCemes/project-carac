from dataclasses import dataclass
from sys import argv

import matplotlib
import numpy as np
import pandas as pd
from pipeline.dataframe import Columns
from rich.status import Status
from scipy.linalg import norm
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.axes import Axes

from .defs import CONTEXT, DPI_IMAGE, PLOT_PATH, OUTPUT_PATH


# NAME = "parameters-axis"
# LEGENDS = ("Coupled Axis", "Single Axis")
# FILES_1 = (p for d in ["axis-coupled"] for p in OUTPUT_PATH.glob(f"{d}/*.parquet"))
# FILES_2 = (p for d in ["axis-uncoupled"] for p in OUTPUT_PATH.glob(f"{d}/*.parquet"))

NAME = "parameters-free-flight"
LEGENDS = ("Free Flight", "Extended Free Flight")
FILES_1 = (p for d in ["free-flight-3"] for p in OUTPUT_PATH.glob(f"{d}/*.parquet"))
FILES_2 = (
    p for d in ["free-flight-extended"] for p in OUTPUT_PATH.glob(f"{d}/*.parquet")
)


TRIM_ENDS: float | bool = 0.1

VIOLIN_COLOR_1 = "#ee6677"
VIOLIN_COLOR_2 = "#4477aa"


@dataclass
class Parameter:
    col_name: str
    label: str


PARAMETERS_1 = [
    Parameter("alpha", r"$\alpha$"),
    Parameter("beta", r"$\beta$"),
]

PARAMETERS_2 = [
    Parameter("p", r"$p$"),
    Parameter("q", r"$q$"),
    Parameter("r", r"$r$"),
]

PARAMETERS_3 = [
    Parameter("uvw", r"$\lVert\textbf{v}_w\rVert$"),
    # Parameter("uvw", r"$v_w$"),
    # Parameter("v", r"$v$"),
    # Parameter("w", r"$w$"),
]


def main():
    with Status("Loading data"):
        df1 = pd.concat(map(pd.read_parquet, FILES_1), ignore_index=True)
        df2 = (
            pd.concat(map(pd.read_parquet, FILES_2), ignore_index=True)
            if FILES_2
            else None
        )

        if isinstance(TRIM_ENDS, float) and TRIM_ENDS > 0:
            df1.sort_values(by="time", inplace=True)
            df1 = df1.iloc[int(len(df1) * TRIM_ENDS) : int(len(df1) * (1 - TRIM_ENDS))]

            if df2 is not None:
                df2.sort_values(by="time", inplace=True)
                df2 = df2.iloc[
                    int(len(df2) * TRIM_ENDS) : int(len(df2) * (1 - TRIM_ENDS))
                ]

        angle_columns = [c.col_name for c in PARAMETERS_1 + PARAMETERS_2]
        df1[angle_columns] = np.rad2deg(df1[angle_columns])

        df1["uvw"] = norm(df1[Columns.AeroVelocity], axis=1)

        if df2 is not None:
            df2["uvw"] = norm(df2[Columns.AeroVelocity], axis=1)
            df2[angle_columns] = np.rad2deg(df2[angle_columns])

    with Status("Generating plots"):
        matplotlib.rc("text", usetex=True)
        matplotlib.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"
        # plt.style.use(CONTEXT)
        plt.figure(figsize=(5, 3))
        gs = GridSpec(3, 1, height_ratios=[2, 3, 1])

        ax1 = plt.subplot(gs[0])
        ax2 = plt.subplot(gs[1])
        ax3 = plt.subplot(gs[2])

        plot_violin(df1, df2, PARAMETERS_1, ax=ax1)
        plot_violin(df1, df2, PARAMETERS_2, ax=ax2)
        plot_violin(df1, df2, PARAMETERS_3, ax=ax3)

        for ax in [ax1, ax2, ax3]:
            ax.set_xlabel("")
        # ax.tick_params(axis="y", length=0)
        # sns.despine(ax=ax, left=True, bottom=False)

        plt.tight_layout()
        # plt.subplots_adjust(hspace=0.25)
        plt.subplots_adjust(hspace=0.35)

        # ax1.set_ylabel("[degrees]")
        # ax2.set_ylabel("[degrees/s]")
        # ax3.set_ylabel("[m/s]")

        for ax in [ax1, ax2, ax3]:
            ax.set_ylabel("")
            ax.set_yticks([])

        ax3.set_xlim(1.5, 7.5)

        ax1.legend(LEGENDS, loc="lower left", prop={"size": 7})

    if "--tikz" in argv:
        suffix = ".pdf" if "--pdf" in argv else ".svg" if "--svg" in argv else ".eps"
        PLOT_PATH.mkdir(parents=True, exist_ok=True)
        output_file = (PLOT_PATH / NAME).with_suffix(suffix)
        plt.savefig(output_file, dpi=DPI_IMAGE, transparent=True)

    if not "--tikz" in argv:
        plt.show()


def plot_violin(
    df1: pd.DataFrame,
    df2: pd.DataFrame | None,
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
    df = df1_melted

    if df2 is not None:
        df2_melted = df2[columns].copy()
        df2_melted = df2_melted.melt(var_name="Parameter", value_name="Value")
        df2_melted["split_cat"] = "B"

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
        palette=(
            [VIOLIN_COLOR_1, VIOLIN_COLOR_2] if df2 is not None else [VIOLIN_COLOR_1]
        ),
        legend=False,
        linewidth=0,
        density_norm="width",
        cut=0,
        ax=ax,
    )


if __name__ == "__main__":
    main()
