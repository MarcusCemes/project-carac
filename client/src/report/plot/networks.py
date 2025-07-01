from sys import argv
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle
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
from training.networks.lstm import LSTMNet
from training.networks.mlp import MLPNet
from training.defs import (
    INPUT_COLUMNS,
    OUTPUT_COLUMNS,
    LSTM_SEQUENCE_LENGTH,
)

from .defs import *
from .common import find_experiment
from .lib.export import save_figure_tikz


CONTEXT: list[str] = []

try:
    import scienceplots  # type: ignore[import]

    del scienceplots
    # CONTEXT.append("science")

except ImportError:
    pass

DATASET_FREE_FLIGHT = find_experiment("free-flight-3", 10)
DATASET_FREE_FLIGHT_EXT = find_experiment("free-flight-extended", 0)
DATASET_AXIS = find_experiment("axis-coupled", 540)

LABELS = [
    "FF",
    "FFE",
    "AX",
]

MODEL_NAMES = {"model": "Analytical", "axis": "M1", "ff-all": "M3", "ff-": "M2"}

MODEL_DIR: str | None = "250626_223226_normal"

DEVICE = "cuda" if cuda.is_available() else "cpu"
SUFFIXES = ["", "_model", "_mlp", "_lstm"]

MODEL_MAP = {
    "model": "Analytical",
    "axis-mlp": "M1-MLP",
    "axis-lstm": "M1-LSTM",
    "ff-mlp": "M2-MLP",
    "ff-lstm": "M2-LSTM",
    "ff-all-mlp": "M3-MLP",
    "ff-all-lstm": "M3-LSTM",
}

MODEL_ORDER = [
    "M1-MLP",
    "M1-LSTM",
    "M2-MLP",
    "M2-LSTM",
    "M3-MLP",
    "M3-LSTM",
]

HEAT_ORDER = ["Analytical", *MODEL_ORDER]

HEATMAP_SIZE = (TIKZ_SIZE[0], 2)

Models = dict[str, MLPNet | LSTMNet]


def main():
    if not "--no-tex" in argv:
        plt.style.use(CONTEXT)

    with Status("Loading data..."):
        dfs = [
            read_parquet(DATASET_FREE_FLIGHT),
            read_parquet(DATASET_FREE_FLIGHT_EXT),
            read_parquet(DATASET_AXIS),
        ]

    with Status(f"Loading models..."):
        models = load_models()

    if not models:
        rprint("[red]No models found! [/]")
        return

    # == Inference == #

    with Status("Evaluating models..."):
        for name, model in models.items():
            for df in dfs:
                if isinstance(model, MLPNet):
                    evaluate_model_mlp(name, model, df)

                else:
                    evaluate_model_lstm(name, model, df)

                compute_aero(df, name)

    # == Evaluation == #

    df_rmse = DataFrame(iter_rmse(dfs, models))

    rprint(generate_rmse_table(df_rmse))

    PLOT_PATH.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=HEATMAP_SIZE)
    plt.figure(figsize=(7, 3))  # For SVG
    plot_heatmap(df_rmse)
    plt.subplots_adjust(bottom=0.25)  # For SVG
    plt.savefig(PLOT_PATH / "free_flight_heatmap.svg", dpi=DPI_IMAGE, transparent=True)

    fig, axs = plt.subplots(2, 2, sharex=True, figsize=TIKZ_SIZE_XL)
    fig.suptitle(f"Evaluation on Free Flight Datasets")

    for df, row in zip(dfs, axs):
        row[0].set_ylabel("Lift (N)")
        plot_models(df, "lift", models, row[0])

        row[1].set_ylabel("Drag (N)")
        plot_models(df, "drag", models, row[1])

    # for ax in axs.flat:
    #     ax.set_xlim(16, 26)

    fig.savefig(
        PLOT_PATH / "free_flight_evaluation.svg", dpi=DPI_IMAGE, transparent=True
    )

    _, axs = plt.subplots(2, 1)
    axs[0].set_ylabel("Lift (N)")
    plot_models(dfs[2], "lift", models, axs[0])

    axs[1].set_ylabel("Drag (N)")
    plot_models(dfs[2], "drag", models, axs[1])

    if "--tikz" in argv:
        plt.figure(figsize=HEATMAP_SIZE)

        plot_heatmap(df_rmse, ax=plt.gca())

        plt.xticks(rotation=20)
        plt.yticks(rotation=0)
        plt.xlabel("")
        plt.ylabel("")
        plt.tick_params(
            axis="both",
            which="both",
            left=False,
            right=False,
            bottom=False,
            top=False,
        )

        save_figure_tikz(PLOT_PATH / "model_heatmap", show_axes=True)
        plt.close()

        plt.figure(figsize=TIKZ_SIZE)
        ax = plt.gca()
        plot_models(dfs[1], "lift", models, ax=ax)

        ax.set_xlim(299.5, 302.5)
        ax.set_ylim(-2.5, 3)
        ax.legend(ncol=4, prop={"size": 8})
        plt.tight_layout()

        save_figure_tikz(PLOT_PATH / "free_flight_comparison")
        plt.close()

    # else:
    #     plt.show()

    plt.figure(figsize=(7, 3))
    ax = plt.gca()
    plot_models(dfs[1], "lift", models, ax=ax)

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Lift [N]")
    ax.set_xlim(299.5, 302.5)
    ax.set_ylim(-2.5, 3)
    ax.legend(ncol=4, prop={"size": 7})
    plt.subplots_adjust(bottom=0.25)
    # plt.tight_layout()

    plt.savefig(
        PLOT_PATH / "free_flight_comparison.svg",
        dpi=DPI_IMAGE,
        transparent=True,
    )
    plt.close()


def generate_rmse_table(df: DataFrame) -> Table:
    table = Table(title="RMSE Results")

    table.add_column("Model")
    table.add_column("Dataset")
    table.add_column("RMSE")

    for row in df.itertuples(index=False):
        table.add_row(
            str(row.Model),
            str(row.Dataset),
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

    df.plot.line(
        x=Columns.Time,
        y=f"{col}_model",
        label="Model",
        linewidth=1,
        style="--",
        ax=ax,
        color="#333333",
    )

    df.plot(
        x=Columns.Time,
        y=[f"{col}_{name}" for name in models],
        label=MODEL_ORDER,
        ax=ax,
        linewidth=0.8,
        alpha=0.6,
        # color=[
        #     "#F0561D",
        #     "#B1380B",
        #     "#63993D",
        #     "#204D00",
        #     "#5E40BE",
        #     "#21134D",
        # ],
        # color=["#e60049", "#0bb4ff", "#50e991", "#e6d800", "#9b19f5", "#ffa300"],
        color=[
            "#b30000",
            "#7c1158",
            "#0d88e6",
            "#00b7c7",
            "#5ad45a",
            "#8be04e",
        ],
    )


def plot_heatmap(df: DataFrame, ax: Axes | None = None) -> None:
    df = df.pivot_table(
        index="Dataset",
        columns="Model",
        values="rmse",
    )

    df = df.rename(columns=MODEL_MAP)
    df = df.reindex(columns=HEAT_ORDER)

    mask = DataFrame(False, index=df.index, columns=df.columns)

    cells_to_hide = [
        ("AX", "M1-MLP"),
        ("AX", "M1-LSTM"),
        ("FF", "M2-MLP"),
        ("FF", "M2-LSTM"),
        ("FF", "M3-MLP"),
        ("FF", "M3-LSTM"),
        ("FFE", "M3-MLP"),
        ("FFE", "M3-LSTM"),
    ]

    for dataset, model in cells_to_hide:
        if dataset in mask.index and model in mask.columns:
            mask.loc[dataset, model] = True

    sns.heatmap(
        df,
        annot=True,
        fmt=".2f",
        cmap="viridis",
        cbar=False,
        ax=ax,
        mask=mask,
    )

    ax = ax if ax is not None else plt.gca()

    for coord in [(1, 0), (3, 1), (5, 1), (5, 2)]:
        # ax.add_patch(Rectangle(coord, 2, 1, fill=False, edgecolor="#ff6361", lw=1.5))
        ax.add_patch(
            Rectangle(
                coord,
                2,
                1,
                fill=True,
                facecolor="#413D3A",
            )
        )


def evaluate_model_mlp(name: str, model: MLPNet, df: DataFrame):
    inputs = tensor(df[INPUT_COLUMNS].to_numpy(), dtype=float32).to(DEVICE)

    with no_grad():
        prediction = model(inputs)  # type: Tensor

    columns = [f"{col}_{name}" for col in OUTPUT_COLUMNS]
    df[columns] = prediction.detach().cpu().numpy()


def evaluate_model_lstm(name: str, model: LSTMNet, df: DataFrame):
    input = tensor(df[INPUT_COLUMNS].to_numpy(), dtype=float32).to(DEVICE)

    # Create overlapping sequences of length LSTM_SEQUENCE_LENGTH
    sequences = input.unfold(dimension=0, size=LSTM_SEQUENCE_LENGTH, step=1)

    # Swap the last two dimensions to match LSTM input requirements
    sequences = sequences.permute(0, 2, 1)

    with no_grad():
        prediction = model(sequences)  # type: Tensor

    columns = [f"{col}_{name}" for col in OUTPUT_COLUMNS]
    results = np.full((len(df), len(OUTPUT_COLUMNS)), np.nan)

    # The first valid index for a prediction is at the end of the first sequence
    start_index = LSTM_SEQUENCE_LENGTH - 1
    results[start_index:] = prediction.detach().cpu().numpy()

    df[columns] = results


def compute_aero(df: DataFrame, name: str):
    force_columns = [f"{col}_{name}" for col in Columns.BodyForce]
    aero_columns = [f"{col}_{name}" for col in Columns.AeroForces]
    add_aero_forces(df, force_columns, aero_columns)


def iter_rmse(dfs: list[DataFrame], models: Models):
    for name in ["model", *models]:
        for df, label in zip(dfs, LABELS):
            yield {
                "Model": name,
                "Dataset": label,
                "rmse": compute_rmse(df, name),
            }


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

    for path in weights:
        model = MLPNet.default() if "mlp" in path.stem else LSTMNet.default()
        model.load_state_dict(torch_load(path))
        model.to(DEVICE)
        model.eval()

        models[path.stem] = model

    return models


def find_last_checkpoint() -> Path:
    checkpoints = [p for p in CHECKPOINT_PATH.iterdir() if p.stem[0:6].isnumeric()]
    checkpoints.sort()

    if not checkpoints:
        raise FileNotFoundError("No checkpoints found in the directory.")

    checkpoint = checkpoints[-1]
    rprint(f"[b]Loaded checkpoint {checkpoint.name}[/]")

    return checkpoint


def model_name(name: str) -> str:
    for prefix, model in MODEL_NAMES.items():
        if name.startswith(prefix):
            if "lstm" in name:
                model += "-LSTM"
            elif "mlp" in name:
                model += "-MLP"

            return model
    raise ValueError(f"Unknown model name: {name}")


if __name__ == "__main__":
    main()
