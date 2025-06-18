from pathlib import Path
from typing import TypedDict

import matplotlib.pyplot as plt


DATA_PATH = (Path(__file__).parent / "../../../../data-old").resolve()

INPUT_PATH = DATA_PATH / "input"
OUTPUT_PATH = DATA_PATH / "output"
PLOT_PATH = DATA_PATH / "plots"


DPI_IMAGE = 600
TIKZ_SIZE = (6, 4)

plt.style.use("seaborn-v0_8")
plt.rcParams["figure.figsize"] = (8, 6)
plt.rcParams["figure.dpi"] = 100
plt.rcParams["scatter.edgecolors"] = "none"


class PlotOpts(TypedDict, total=False):
    x: str
    xlabel: str
