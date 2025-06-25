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
from training.networks.lstm import LSTMNet
from training.networks.mlp import MLPNet
from training.defs import (
    INPUT_COLUMNS,
    OUTPUT_COLUMNS,
    LSTM_SEQUENCE_LENGTH,
)

from .defs import *
from .common import find_experiment


DATASET_FREE_FLIGHT = find_experiment("free-flight-3", 10)
DATASET_FREE_FLIGHT_EXT = find_experiment("free-flight-extended", 0)
DATASET_AXIS = find_experiment("axis-coupled", 540)

LABELS = [
    "Free Flight",
    "Free Flight Extended",
    "Axis Coupled",
]

MODEL_DIR: str | None = "250625_224126"

DEVICE = "cuda" if cuda.is_available() else "cpu"
SUFFIXES = ["", "_model", "_mlp", "_lstm"]


Models = dict[str, MLPNet | LSTMNet]


def main():
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

    with Status("Evaluating models..."):
        for name, model in models.items():
            for df in dfs:
                if isinstance(model, MLPNet):
                    evaluate_model_mlp(name, model, df)

                else:
                    evaluate_model_lstm(name, model, df)

                compute_aero(df, name)

    rmse_df = DataFrame(iter_rmse(dfs, models))

    table = Table(title="RMSE Results")
    table.add_column("Model")
    table.add_column("Dataset")
    table.add_column("Lift")
    table.add_column("Drag")
    table.add_column("Side")

    for row in rmse_df.itertuples(index=False):
        table.add_row(
            str(row.model),
            str(row.dataset),
            f"{row.lift:.3f}",
            f"{row.drag:.3f}",
            f"{row.side_force:.3f}",
        )

    rprint(table)

    fig, axs = plt.subplots(1, 3)

    for ax, values in zip(axs, ["lift", "drag", "side_force"]):
        sns.heatmap(
            rmse_df.pivot_table(index="model", columns="dataset", values=values),
            ax=ax,
            square=True,
            annot=True,
            fmt=".3f",
        )

    for df, label in zip(dfs, LABELS):
        fig, axs = plt.subplots(1, 2)

        plot_models(df, "lift", models, axs[0])
        plot_models(df, "drag", models, axs[1])

        fig.suptitle(f"Evaluation on {label}")

    plt.show()


def plot_models(df: DataFrame, col: str, models: Models, ax: Axes):
    df.plot.scatter(x=Columns.Time, y=col, label="Data", s=1, ax=ax)
    df.plot.line(x=Columns.Time, y=f"{col}_model", label="Model", ax=ax)

    for name in models.keys():
        df.plot.line(
            x=Columns.Time,
            y=f"{col}_{name}",
            label=name,
            style="--",
            ax=ax,
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
    for name in models:
        for df, label in zip(dfs, LABELS):
            rmse = compute_rmse(df, name)

            yield {
                "model": name,
                "dataset": label,
                "lift": rmse["lift"],
                "drag": rmse["drag"],
                "side_force": rmse["side_force"],
            }


def compute_rmse(df: DataFrame, name: str) -> dict[str, float]:
    pred_cols = [f"{col}_{name}" for col in Columns.AeroForces]

    eval_df = df[Columns.AeroForces + pred_cols].copy()

    # Drop any rows with NaN values
    eval_df.dropna(inplace=True)

    if eval_df.empty:
        raise ValueError("The evaluation DataFrame is empty after dropping NaN values.")

    y_true = eval_df[Columns.AeroForces].to_numpy()
    y_pred = eval_df[pred_cols].to_numpy()

    rmse_per_dim = {}

    for i, col_name in enumerate(Columns.AeroForces):
        dim_mse = mean_squared_error(y_true[:, i], y_pred[:, i])
        rmse_per_dim[col_name] = np.sqrt(dim_mse).item()

    return rmse_per_dim


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
    checkpoints = [*CHECKPOINT_PATH.iterdir()]
    checkpoints.sort()

    if not checkpoints:
        raise FileNotFoundError("No checkpoints found in the directory.")

    checkpoint = checkpoints[-1]
    rprint(f"[b]Loaded checkpoint {checkpoint.name}[/]")

    return checkpoint


if __name__ == "__main__":
    main()
