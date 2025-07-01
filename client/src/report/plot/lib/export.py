from pathlib import Path
from sys import argv

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.legend import Legend
import numpy as np


# def save_legend(legend: Legend, path: Path, expand=[-5, -5, 5, 5]):
#     fig = legend.figure
#     fig.canvas.draw()

#     bbox = legend.get_window_extent()
#     bbox = bbox.from_extents(*(bbox.extents + np.array(expand)))
#     bbox = bbox.transformed(fig.dpi_scale_trans.inverted())

#     fig.savefig(  # type: ignore
#         path,
#         dpi="figure",
#         bbox_inches=bbox,
#         transparent=True,
#     )


def save_legend(ax: Axes, path: Path, **kwargs):
    handles, labels = ax.get_legend_handles_labels()

    legend_fig, legend_ax = plt.subplots(figsize=(6, 1))
    legend_ax.axis("off")
    legend_ax.legend(handles, labels, loc="center", **kwargs)

    legend_fig.savefig(
        path,
        bbox_inches="tight",
        dpi="figure",
        facecolor="white",
        edgecolor="none",
        transparent=True,
    )


def save_figure_tikz(
    path: Path,
    fig: Figure | None = None,
    ax: Axes | None = None,
    show_axes: bool = False,
) -> None:
    if fig is None:
        fig = plt.gcf()

    if ax is None:
        ax = fig.gca()

    if not show_axes:
        ax.axis("off")

    suffix = ".pdf" if "--pdf" in argv else ".svg" if "--svg" in argv else ".eps"

    print("Saving TikZ figure to", path)
    fig.savefig(
        path.with_suffix(suffix),
        bbox_inches="tight",
        pad_inches=0,
        transparent=True,
    )

    _save_tikz_code(ax, path)


def _save_tikz_code(ax: Axes, path: Path):
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    path = path.with_suffix(".tex")

    content = f"""\\begin{{tikzpicture}}
    \\begin{{axis}}[
        xmin={xlim[0]}, xmax={xlim[1]}, ymin={ylim[0]}, ymax={ylim[1]},
        xlabel={{{ax.get_xlabel().replace("_", r"\_")}}},
        ylabel={{{ax.get_ylabel().replace("_", r"\_")}}},
        axis line style={{-}},
        enlarge x limits=false,
        enlarge y limits=false,
        width=\\columnwidth,
        height=0.667\\columnwidth
    ]
    \\addplot graphics [
        xmin={xlim[0]}, xmax={xlim[1]}, ymin={ylim[0]}, ymax={ylim[1]}
    ] {{{path.with_suffix(".eps").name}}};
    \\end{{axis}}
\\end{{tikzpicture}}
"""

    with open(path, "w", encoding="utf-8") as f:
        f.write(content.strip())
