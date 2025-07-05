from pathlib import Path

from .defs import OUTPUT_PATH


def find_experiment(name: str, id: int) -> Path:
    dir = OUTPUT_PATH / name

    if not dir.exists():
        raise FileNotFoundError(f"Experiment directory '{dir}' does not exist.")

    for path in dir.glob(f"{id:04d}_*.parquet"):
        if path.is_file():
            return path

    raise FileNotFoundError(f"No file matching '{id:04d}_*.parquet' found in '{dir}'.")
