from random import shuffle
from bisect import bisect_right
from itertools import accumulate

from pandas import read_parquet
from torch import float32, tensor, Tensor
from torch.utils.data import Dataset
from rich import print as rprint

from .defs import *

Batch = tuple[Tensor, Tensor]


class SequentialFlightDataset(Dataset[Batch]):
    """
    A dataset that loads flight data and serves it as sequences using a sliding window.
    Sequences are created per-file and do not cross experiment boundaries.
    This is suitable for both LSTM and MLP models.
    """

    def __init__(self, datasets: list[str]):
        super().__init__()

        all_data = _load_dataframes(datasets)

        self._data_per_file = []
        sequences_per_file = []

        # Filter out files that are too short and calculate valid sequence counts for the rest
        for inputs, outputs in all_data:
            if inputs.shape[0] >= LSTM_SEQUENCE_LENGTH:
                self._data_per_file.append((inputs, outputs))
                num_sequences = inputs.shape[0] - LSTM_SEQUENCE_LENGTH + 1
                sequences_per_file.append(num_sequences)

        if not self._data_per_file:
            raise ValueError(
                f"No data files for {datasets} are long enough for a sequence of length {LSTM_SEQUENCE_LENGTH}."
            )

        self._num_samples = sum(sequences_per_file)

        # Create a lookup table for mapping a global index to a file and a local index
        self._cumulative_sequences = list(accumulate(sequences_per_file))

    def __len__(self) -> int:
        return self._num_samples

    def __getitem__(self, idx: int) -> Batch:
        if idx < 0 or idx >= self._num_samples:
            raise IndexError("Index out of range")

        # Find which file this index belongs to using binary search
        file_idx = bisect_right(self._cumulative_sequences, idx)

        # Determine the starting index within that specific file's data.
        local_start_idx = idx
        if file_idx > 0:
            local_start_idx -= self._cumulative_sequences[file_idx - 1]

        inputs, outputs = self._data_per_file[file_idx]

        # Create the sequence from the specific file's data
        end_idx = local_start_idx + LSTM_SEQUENCE_LENGTH
        input_sequence = inputs[local_start_idx:end_idx]

        output_wrench = outputs[end_idx - 1]

        return (input_sequence, output_wrench)


class FreeFlightDataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__(["free-flight-3"])


class FreeFlightExtendedDataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__(["free-flight-extended"])


class AxisDataset(SequentialFlightDataset):
    def __init__(self):
        super().__init__(["axis-uncoupled", "axis-coupled"])


def _load_dataframes(names: list[str]) -> list[tuple[Tensor, Tensor]]:
    """
    Loads all parquet files from a directory into a list of tensor pairs,
    with each pair representing one file.
    """

    files = [
        file
        for name in names
        for file in (INPUT_DIR / name).iterdir()
        if file.suffix == ".parquet"
    ]

    if not files:
        raise ValueError(f"No .parquet files for {names}")

    shuffle(files)

    data_list = []
    total_samples = 0

    for file in files:
        df = read_parquet(file)

        if not df.empty:
            tensor_inputs = tensor(df[INPUT_COLUMNS].to_numpy(), dtype=float32)
            tensor_outputs = tensor(df[OUTPUT_COLUMNS].to_numpy(), dtype=float32)
            data_list.append((tensor_inputs, tensor_outputs))
            total_samples += len(df)

    if not data_list:
        raise ValueError(f"Data data found for {names}")

    rprint(
        f"📔 Loaded {len(data_list)} files from [bold white]{names}[/] ({total_samples} total samples)"
    )

    return data_list
