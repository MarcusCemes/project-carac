from sys import argv
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
import numpy as np
from pandas import DataFrame, read_parquet
from rich.table import Table
from torch import Tensor, cuda, float32, load as torch_load, no_grad, tensor
from rich import print as rprint
from rich.status import Status
import seaborn as sns
from sklearn.metrics import mean_squared_error

from pipeline.dataframe import Columns
from pipeline.process import add_aero_forces
from training.networks.mlp import MLPNet
from training.defs import (
    INPUT_COLUMNS,
    MLP_MULTI_LAYER_SIZES,
    OUTPUT_COLUMNS,
)

from .defs import *
from .common import find_experiment
from .lib.export import save_figure_tikz


CONTEXT: list[str] = []

try:
    import scienceplots  # type: ignore[import]

    del scienceplots
    CONTEXT.append("science")

except ImportError:
    pass


DATASET_FREE_FLIGHT_EXT = find_experiment("free-flight-extended", 0)
MODEL_DIR: str | None = None

DEVICE = "cuda" if cuda.is_available() else "cpu"
HEATMAP_SIZE = (TIKZ_SIZE[0], 2)

Models = dict[str, MLPNet]


def main():
    if not "--no-tex" in argv:
        plt.style.use(CONTEXT)

    with Status("Loading data..."):
        df = read_parquet(DATASET_FREE_FLIGHT_EXT)

    with Status(f"Loading models..."):
        models = load_models()

    if not models:
        rprint("[red]No models found! [/]")
        return

    # == Inference == #

    with Status("Evaluating models..."):
        for name, model in models.items():
            evaluate_model_mlp(name, model, df)
            compute_aero(df, name)

    # == Evaluation == #

    df_rmse = DataFrame(iter_rmse(df, models))

    rprint(generate_rmse_table(df_rmse))

    PLOT_PATH.mkdir(parents=True, exist_ok=True)

    fig, axs = plt.subplots(1, 2, sharex=True, figsize=TIKZ_SIZE_XL)
    fig.suptitle(f"Evaluation on Free Flight Datasets")

    axs[0].set_ylabel("Lift (N)")
    plot_models(df, "lift", models, axs[0])

    axs[1].set_ylabel("Drag (N)")
    plot_models(df, "drag", models, axs[1])

    if "--tikz" in argv:
        plt.figure(figsize=TIKZ_SIZE)
        ax = plt.gca()
        plot_models(df, "lift", models, ax=ax)

        ax.set_xlim(317, 322)
        ax.set_ylim(-2.5, 2.8)
        ax.legend(ncol=4, prop={"size": 8})
        plt.tight_layout()

        save_figure_tikz(PLOT_PATH / "network_complexity")
        plt.close()

    plt.figure(figsize=(7, 3))
    ax = plt.gca()
    plot_models(df, "lift", models, ax=ax)

    ax.set_ylabel("Lift [N]")
    ax.set_xlabel("Time [s]")
    ax.set_xlim(317, 322)
    ax.set_ylim(-2.5, 2.8)
    ax.legend(ncol=4, prop={"size": 8})
    plt.subplots_adjust(bottom=0.25)

    plt.savefig(
        PLOT_PATH / "network_complexity.svg",
        transparent=True,
    )
    plt.close()


def generate_rmse_table(df: DataFrame) -> Table:
    table = Table(title="RMSE Results")

    table.add_column("Model")
    table.add_column("RMSE")

    for row in df.itertuples(index=False):
        table.add_row(
            str(row.Model),
            f"{row.rmse:.3f}",
        )

    return table


def plot_models(df: DataFrame, col: str, models: Models, ax: Axes):
    df.plot.line(
        x=Columns.Time,
        y=col,
        label="Data",
        linewidth=1.5,
        ax=ax,
        color="#000000",
    )

    df.plot(
        x=Columns.Time,
        y=[f"{col}_{name}" for name in models],
        label=models,
        ax=ax,
        linewidth=0.8,
        alpha=0.6,
        color=[
            "#b30000",
            "#7c1158",
            "#4421af",
            "#1a53ff",
            "#0d88e6",
            "#00b7c7",
            "#5ad45a",
            "#8be04e",
            "#ebdc78",
        ],
    )


def plot_heatmap(df: DataFrame, ax: Axes | None = None) -> None:
    df_loss = df.pivot_table(
        index="Dataset",
        columns="Model",
        values="rmse",
    )

    # df_loss = df_loss.reindex(columns=PLOT_ORDER)

    sns.heatmap(
        df_loss,
        annot=True,
        fmt=".2f",
        cmap="viridis",
        cbar=False,
        # cbar_kws={"shrink": 0.4},
        ax=ax,
    )


def evaluate_model_mlp(name: str, model: MLPNet, df: DataFrame):
    inputs = tensor(df[INPUT_COLUMNS].to_numpy(), dtype=float32).to(DEVICE)

    with no_grad():
        prediction = model(inputs)  # type: Tensor

    columns = [f"{col}_{name}" for col in OUTPUT_COLUMNS]
    df[columns] = prediction.detach().cpu().numpy()


def compute_aero(df: DataFrame, name: str):
    force_columns = [f"{col}_{name}" for col in Columns.BodyForce]
    aero_columns = [f"{col}_{name}" for col in Columns.AeroForces]
    add_aero_forces(df, force_columns, aero_columns)


def iter_rmse(df: DataFrame, models: Models):
    for name in models:
        yield {"Model": name, "rmse": compute_rmse(df, name)}


def compute_rmse(df: DataFrame, model_name: str) -> float:
    cols = ["lift"]
    pred_cols = [f"{col}_{model_name}" for col in cols]

    eval_df = df[cols + pred_cols].copy()

    # Drop any rows with NaN values
    eval_df.dropna(inplace=True)

    if eval_df.empty:
        raise ValueError("The evaluation DataFrame is empty after dropping NaN values.")

    y_true = eval_df[cols].to_numpy()
    y_pred = eval_df[pred_cols].to_numpy()

    rmse_values = []

    for i in range(len(cols)):
        dim_mse = mean_squared_error(y_true[:, i], y_pred[:, i])
        rmse_values.append(np.sqrt(dim_mse).item())

    return np.mean(rmse_values).item()


def load_models() -> Models:
    checkpoint_dir = (
        (CHECKPOINT_PATH / MODEL_DIR)
        if MODEL_DIR is not None
        else find_last_checkpoint()
    )

    weights = checkpoint_dir.glob("*.pth")
    models: Models = {}

    for path, size, letter in zip(
        weights, MLP_MULTI_LAYER_SIZES, "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    ):
        model = MLPNet(size)
        model.load_state_dict(torch_load(path))
        model.to(DEVICE)
        model.eval()

        name = f"{letter}-{",".join(map(str, size))}"
        models[name] = model

    return models


def find_last_checkpoint() -> Path:
    checkpoints = [p for p in CHECKPOINT_PATH.iterdir() if p.stem[0:6].isnumeric()]
    checkpoints.sort()

    if not checkpoints:
        raise FileNotFoundError("No checkpoints found in the directory.")

    checkpoint = checkpoints[-1]
    rprint(f"[b]Loaded checkpoint {checkpoint.name}[/]")

    return checkpoint


if __name__ == "__main__":
    main()
