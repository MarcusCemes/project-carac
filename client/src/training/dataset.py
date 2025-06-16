from pathlib import Path

from pandas import DataFrame, concat, read_parquet
from rich import print as rprint
from scipy.spatial.transform import Rotation
from torch import float32, tensor, Tensor
from torch.utils.data import Dataset

from .defs import *


class FreeFlightDataset(Dataset[tuple[Tensor, Tensor]]):

    def __init__(self, files: list[Path]):
        super().__init__()

        dfs = [load_parquet(file) for file in files]

        if not dfs:
            raise ValueError("No data files found or all files are empty.")

        data = concat(dfs, ignore_index=True)
        rprint(f"Loaded {len(data)} samples from {len(files)} files")

        self._inputs = tensor(data[INPUT_COLUMNS].values, dtype=float32)
        self._outputs = tensor(data[OUTPUT_COLUMNS].values, dtype=float32)

    def __len__(self):
        return self._inputs.shape[0]

    def __getitem__(self, idx):
        return self._inputs[idx], self._outputs[idx]


def load_parquet(path: Path) -> DataFrame:
    df = read_parquet(path)

    rotation = Rotation.from_euler(EULER_ORDER, df[ATTITUDE_COLUMNS].to_numpy())
    df[ATTITUDE_QUAT_COLUMNS] = rotation.as_quat()

    return df
