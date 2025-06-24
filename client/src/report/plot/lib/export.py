from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure


def save_figure_tikz(path: Path, fig: Figure | None = None, ax: Axes | None = None):
    if fig is None:
        fig = plt.gcf()

    if ax is None:
        ax = fig.gca()

    ax.axis("off")

    print("Saving TikZ figure to", path)
    fig.savefig(
        path.with_suffix(".eps"),
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
