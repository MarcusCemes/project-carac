from pandas import concat, read_parquet
from torch import float32, tensor, Tensor
from torch.utils.data import Dataset
from rich.status import Status
from rich import print as rprint

from .defs import *

Batch = tuple[Tensor, Tensor, Tensor]


class SequentialFlightDataset(Dataset[Batch]):
    """
    A dataset that loads flight data and serves it as sequences using a sliding window.
    This is suitable for both LSTM and MLP models.
    """

    def __init__(self, data_name: str):
        super().__init__()

        (self._inputs, self._outputs, self._residuals) = _load_dataframes(data_name)
        self._num_samples = self._inputs.shape[0] - LSTM_SEQUENCE_LENGTH + 1

    def __len__(self) -> int:
        return self._num_samples

    def __getitem__(self, idx: int) -> Batch:
        end_idx = idx + LSTM_SEQUENCE_LENGTH
        input_sequence = self._inputs[idx:end_idx]

        target_idx = end_idx - 1
        output_wrench = self._outputs[target_idx]
        output_residual = self._residuals[target_idx]

        return (input_sequence, output_wrench, output_residual)


class FreeFlightDataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__("free-flight")


class FreeFlight2Dataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__("free-flight-2")


class AxisDataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__("axis-uncoupled")


def _load_dataframes(name: str) -> tuple[Tensor, Tensor, Tensor]:
    with Status("Loading data..."):
        count = 0

        def inc(file: Path) -> Path:
            nonlocal count
            count += 1
            return file

        df = concat(
            read_parquet(inc(file))
            for file in (INPUT_DIR / name).iterdir()
            if file.suffix == ".parquet"
        )

        if df.empty:
            raise ValueError(f"No data found in {INPUT_DIR / name}")

        tensor_inputs = tensor(df[INPUT_COLUMNS].to_numpy(), dtype=float32)
        tensor_outputs = tensor(df[OUTPUT_COLUMNS].to_numpy(), dtype=float32)

        true_values = df[OUTPUT_COLUMNS].to_numpy()
        model_values = df[MODEL_COLUMNS].to_numpy()
        tensor_residuals = tensor(true_values - model_values, dtype=float32)

        rprint(f"Loaded {count} files with {len(df)} samples from {INPUT_DIR / name}")

    return (tensor_inputs, tensor_outputs, tensor_residuals)
