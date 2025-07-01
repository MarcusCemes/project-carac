from pathlib import Path
from typing import TypedDict

import matplotlib.pyplot as plt


DATA_PATH = (Path(__file__).parent / "../../../../data").resolve()

INPUT_PATH = DATA_PATH / "unprocessed"
OUTPUT_PATH = DATA_PATH / "processed"
CHECKPOINT_PATH = DATA_PATH / "checkpoints"
PLOT_PATH = DATA_PATH / "plots"


DPI_IMAGE = 600
TIKZ_SIZE = (5, 3)
TIKZ_SIZE_L = (8, 6)
TIKZ_SIZE_XL = (12, 8)

plt.rcParams["figure.figsize"] = (8, 6)
plt.rcParams["figure.dpi"] = 100
plt.rcParams["scatter.edgecolors"] = "none"


class PlotOpts(TypedDict, total=False):
    x: str
    xlabel: str
