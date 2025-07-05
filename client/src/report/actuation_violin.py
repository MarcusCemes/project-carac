import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

from .defs import *


FILES_1 = (p for d in ["free-flight-3"] for p in OUTPUT_PATH.glob(f"{d}/*.parquet"))
FILES_2 = (p for d in ["free-flight-3-ext"] for p in OUTPUT_PATH.glob(f"{d}/*.parquet"))


COLUMNS = [
    "drone/motor",
    "drone/left_wing",
    "drone/right_wing",
    "drone/elevator",
    "drone/rudder",
]


def main():
    df1 = pd.concat(map(pd.read_parquet, FILES_1), ignore_index=True)
    df2 = pd.concat(map(pd.read_parquet, FILES_2), ignore_index=True)

    plot_violin(df1, df2, COLUMNS)

    plt.show()


def plot_violin(
    df1: pd.DataFrame,
    df2: pd.DataFrame,
    columns: list[str],
):

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

    sns.violinplot(
        df,
        x="Value",
        y="Parameter",
        hue="split_cat",
        split=True,
        orient="h",
        inner=None,
        legend=False,
        linewidth=0,
        density_norm="width",
        cut=0,
    )


if __name__ == "__main__":
    main()
