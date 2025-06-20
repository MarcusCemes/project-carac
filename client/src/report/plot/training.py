import numpy as np
from pandas import DataFrame, read_parquet
from pipeline.process import add_aero_forces
import torch
from rich import print as rprint
from rich.status import Status

from pipeline.dataframe import Columns
from report.plot.common import find_experiment
from training.networks.lstm import LSTMNet
from training.networks.mlp import MLP
from training.defs import (
    FIG_SIZE,
    INPUT_COLUMNS,
    OUTPUT_COLUMNS,
    MLP_HIDDEN_LAYERS,
    LSTM_HIDDEN_SIZE,
    LSTM_NUM_LAYERS,
    LSTM_SEQUENCE_LENGTH,
)

from .defs import *


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"

    with Status(f"Loading models and data..."):
        model_mlp_w, model_lstm_w, model_mlp_r, model_lstm_r = load_models(device)
        df = read_parquet(OUTPUT_PATH / "free-flight-2-validation.parquet")

    with Status("Evaluating models..."):
        evaluate_model_mlp(model_mlp_w, df, device)
        evaluate_model_lstm(model_lstm_w, df, device)

        compute_aero(df, "mlp")
        compute_aero(df, "lstm")

    _, axs = plt.subplots(2, 1, figsize=FIG_SIZE, sharex=True)

    df.plot(
        x=Columns.Time,
        y=[
            "lift",
            "lift_model",
            "lift_mlp",
            "lift_lstm",
        ],
        ax=axs[0],
    )

    df.plot(
        x=Columns.Time,
        y=Columns.DroneActuators,
        ax=axs[1],
    )

    plt.show()


def evaluate_model_mlp(model: MLP, df: DataFrame, device: str):
    inputs = torch.tensor(df[INPUT_COLUMNS].to_numpy(), dtype=torch.float32).to(device)

    with torch.no_grad():
        prediction = model(inputs)  # type: torch.Tensor

    columns = [f"{col}_mlp" for col in OUTPUT_COLUMNS]
    df[columns] = prediction.detach().cpu().numpy()


def evaluate_model_lstm(model: LSTMNet, df: DataFrame, device: str):
    input = torch.tensor(df[INPUT_COLUMNS].to_numpy(), dtype=torch.float32).to(device)

    # Create overlapping sequences of length LSTM_SEQUENCE_LENGTH
    sequences = input.unfold(dimension=0, size=LSTM_SEQUENCE_LENGTH, step=1)

    # Swap the last two dimensions to match LSTM input requirements
    sequences = sequences.permute(0, 2, 1)

    with torch.no_grad():
        prediction = model(sequences)  # type: torch.Tensor

    columns = [f"{col}_lstm" for col in OUTPUT_COLUMNS]
    results = np.full((len(df), len(OUTPUT_COLUMNS)), np.nan)

    # The first valid index for a prediction is at the end of the first sequence
    start_index = LSTM_SEQUENCE_LENGTH - 1
    results[start_index:] = prediction.detach().cpu().numpy()

    df[columns] = results


def compute_aero(df: DataFrame, name: str):
    force_columns = [f"{col}_{name}" for col in OUTPUT_COLUMNS]
    aero_columns = [f"{col}_{name}" for col in Columns.AeroForces]
    add_aero_forces(df, force_columns, aero_columns)


def load_models(device: str):
    last_checkpoint_dir = find_last_checkpoint()

    models = (init_model_mlp(), init_model_lstm(), init_model_mlp(), init_model_lstm())

    for model, name in zip(models, ["mlp-w", "lstm-w", "mlp-r", "lstm-r"]):
        model.load_state_dict(torch.load(last_checkpoint_dir / f"{name}.pth"))
        model.to(device)
        model.eval()

    return models


def init_model_mlp():
    return MLP(
        input_size=len(INPUT_COLUMNS),
        output_size=len(OUTPUT_COLUMNS),
        hidden_sizes=MLP_HIDDEN_LAYERS,
    )


def init_model_lstm():
    return LSTMNet(
        input_size=len(INPUT_COLUMNS),
        hidden_size=LSTM_HIDDEN_SIZE,
        num_layers=LSTM_NUM_LAYERS,
        output_size=len(OUTPUT_COLUMNS),
    )


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
