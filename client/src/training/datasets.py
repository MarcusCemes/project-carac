from random import shuffle

from pandas import concat, read_parquet
from torch import float32, tensor, Tensor
from torch.utils.data import Dataset
from rich.status import Status
from rich import print as rprint

from .defs import *

Batch = tuple[Tensor, Tensor]


class SequentialFlightDataset(Dataset[Batch]):
    """
    A dataset that loads flight data and serves it as sequences using a sliding window.
    This is suitable for both LSTM and MLP models.
    """

    def __init__(self, data_name: str):
        super().__init__()

        (self._inputs, self._outputs) = _load_dataframes(data_name)
        self._num_samples = self._inputs.shape[0] - LSTM_SEQUENCE_LENGTH + 1

    def __len__(self) -> int:
        return self._num_samples

    def __getitem__(self, idx: int) -> Batch:
        end_idx = idx + LSTM_SEQUENCE_LENGTH
        input_sequence = self._inputs[idx:end_idx]

        target_idx = end_idx - 1
        output_wrench = self._outputs[target_idx]

        return (input_sequence, output_wrench)


class FreeFlightExtended(SequentialFlightDataset):
    def __init__(self):
        super().__init__("free-flight")


class FreeFlight3Dataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__("free-flight-3")


class AxisDataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__("axis-uncoupled")


def _load_dataframes(name: str) -> tuple[Tensor, Tensor]:
    files = [file for file in (INPUT_DIR / name).iterdir() if file.suffix == ".parquet"]

    shuffle(files)
    df = concat(map(read_parquet, files), ignore_index=True)

    if df.empty:
        raise ValueError(f"No data found in {INPUT_DIR / name}")

    tensor_inputs = tensor(df[INPUT_COLUMNS].to_numpy(), dtype=float32)
    tensor_outputs = tensor(df[OUTPUT_COLUMNS].to_numpy(), dtype=float32)

    rprint(f"📔 Found {len(files)} files in [bold white]{name}[/] ({len(df)} samples)")

    return (tensor_inputs, tensor_outputs)
