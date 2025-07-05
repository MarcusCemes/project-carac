import numpy as np
import pandas as pd
from matplotlib.axes import Axes

from ..defs import LINE_WIDTH, POINT_ALPHA, POINT_SIZE


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


def plot_aero_force(
    df: pd.DataFrame,
    name: str,
    colors: dict,
    x_axis_col: str,
    y_axis_col: str,
    y_model_col: str,
    *,
    legend: bool,
    ax: Axes,
    forward_label: str = "F",
    back_label: str = "B",
    plot_scatter: bool = True,
    plot_line: bool = True,
):
    x_deg_col = f"{x_axis_col}_deg"
    df[x_deg_col] = np.rad2deg(df[x_axis_col])

    mid_point = len(df) // 2
    df_f = df.iloc[:mid_point]
    df_b = df.iloc[mid_point:]

    if plot_scatter:
        df_f.plot(
            kind="scatter",
            x=x_deg_col,
            y=y_axis_col,
            color=colors["forward"],
            s=POINT_SIZE,
            alpha=POINT_ALPHA,
            label=f"{name} ({forward_label})",
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
            label=f"{name} ({back_label})",
            ax=ax,
            legend=legend,
        )

    if plot_line:
        df_f_sorted = df_f.sort_values(by=x_deg_col)
        df_b_sorted = df_b.sort_values(by=x_deg_col)

        df_f_sorted.plot(
            kind="line",
            x=x_deg_col,
            y=y_model_col,
            color=colors["model_fwd"],
            linewidth=LINE_WIDTH,
            label=f"Model {name} ({forward_label})",
            ax=ax,
            legend=legend,
        )

        df_b_sorted.plot(
            kind="line",
            x=x_deg_col,
            y=y_model_col,
            color=colors["model_back"],
            linewidth=LINE_WIDTH,
            label=f"Model {name} ({back_label})",
            ax=ax,
            legend=legend,
        )


def plot_experiment(
    df1: pd.DataFrame,
    df2: pd.DataFrame,
    experiment,
    labels,
    ax1: Axes,
    ax2: Axes,
    y_axis_col_1: str,
    y_axis_col_2: str,
    y_axis_col_1_model: str,
    y_axis_col_2_model: str,
    legend: bool,
    forward_label: str = "F",
    back_label: str = "B",
):
    plot_aero_force(
        df1,
        labels[0],
        COLOURS_1,
        experiment.x_axis,
        y_axis_col=y_axis_col_1,
        y_model_col=y_axis_col_1_model,
        ax=ax1,
        legend=legend,
        forward_label=forward_label,
        back_label=back_label,
    )

    plot_aero_force(
        df2,
        labels[1],
        COLOURS_2,
        experiment.x_axis,
        y_axis_col=y_axis_col_1,
        y_model_col=y_axis_col_1_model,
        ax=ax1,
        legend=legend,
        forward_label=forward_label,
        back_label=back_label,
    )

    plot_aero_force(
        df1,
        labels[0],
        COLOURS_1,
        experiment.x_axis,
        y_axis_col=y_axis_col_2,
        y_model_col=y_axis_col_2_model,
        ax=ax2,
        legend=legend,
        forward_label=forward_label,
        back_label=back_label,
    )

    plot_aero_force(
        df2,
        labels[1],
        COLOURS_2,
        experiment.x_axis,
        y_axis_col=y_axis_col_2,
        y_model_col=y_axis_col_2_model,
        ax=ax2,
        legend=legend,
        forward_label=forward_label,
        back_label=back_label,
    )
