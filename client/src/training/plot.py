from pandas import DataFrame, concat, read_parquet
from torch import float32, load, no_grad, tensor

from .defs import *
from .network import SimpleFeedForwardNN


DATAFRAMES = [
    INPUT_DIR / f"free-flight/{name}"
    for name in ["0004_free-flight_w0.5t-1u1v-1.0.parquet"]
]


def main():
    model = load_model()
    df = concat(map(read_parquet, DATAFRAMES))


def load_model():
    checkpoint = find_last_checkpoint()

    model = SimpleFeedForwardNN(
        len(INPUT_COLUMNS),
        len(OUTPUT_COLUMNS),
        HIDDEN_LAYERS,
    )

    model.load_state_dict(load(checkpoint))
    model.eval()

    return model


def find_last_checkpoint() -> Path:
    checkpoints = [entry for entry in OUTPUT_DIR.glob("*.pth") if entry.is_file()]

    if not checkpoints:
        raise FileNotFoundError("No checkpoints found in the output directory")

    checkpoints.sort()
    return checkpoints[-1]


def evaluate_model(df: DataFrame, model: SimpleFeedForwardNN) -> DataFrame:
    inputs = tensor(df[INPUT_COLUMNS].to_numpy(), dtype=float32)

    # Get predictions without tracking gradients
    with no_grad():
        predictions = model(inputs)

    # Add predictions as new columns to the DataFrame
    df_eval = df.copy()

    # df_eval[C] = predictions[:, 0].numpy()
    raise NotImplementedError("TODO")

    df_eval["drag_pred"] = predictions[:, 1].numpy()

    print("Model evaluation complete. DataFrame augmented with predictions.")
    return df_eval


if __name__ == "__main__":
    main()
