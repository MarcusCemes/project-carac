from pathlib import Path

from .dataset import FreeFlightDataset
from .defs import *
from .train import train


def main():
    input_dir = Path(__file__).parent / DATA_DIR / "input"
    files = list(input_dir.glob("*.parquet"))

    dataset = FreeFlightDataset(files)

    output_dir = Path(__file__).parent / DATA_DIR / "output"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "trained_model.pth"

    train(dataset, output_path)


if __name__ == "__main__":
    main()
