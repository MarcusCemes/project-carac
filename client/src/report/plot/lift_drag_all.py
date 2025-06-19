import pandas as pd
from matplotlib import pyplot as plt

from .common import OUTPUT_PATH
from .defs import DPI_IMAGE, PLOT_PATH
from .lift_drag import COLOURS_1, COLOURS_2, plot_aero_force


def main():

    for dataset in OUTPUT_PATH.iterdir():
        if not dataset.is_dir():
            continue

        print(f"Processing dataset: {dataset.name}")

        for file in dataset.iterdir():
            if not file.is_file() or file.suffix != ".parquet":
                continue

            df = pd.read_parquet(file)
            plot_df(df, dataset.name, file.stem)


def plot_df(df: pd.DataFrame, experiment: str, name: str):
    fig, axs = plt.subplots(2, 2)

    plots = [
        ((0, 0), "alpha", "lift", COLOURS_1),
        ((0, 1), "alpha", "drag", COLOURS_2),
        ((1, 0), "beta", "lift", COLOURS_1),
        ((1, 1), "beta", "drag", COLOURS_2),
    ]

    for coords, x, y, colour in plots:
        plot_aero_force(
            df,
            name,
            colour,
            x,
            y,
            f"{y}_model",
            ax=axs[coords],
        )

    path = PLOT_PATH / f"batch/{experiment}/{name}.png"
    path.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(path, dpi=DPI_IMAGE)
    plt.close(fig)


if __name__ == "__main__":
    main()
