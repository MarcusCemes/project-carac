from time import time

from .dataset import FreeFlightDataset
from .defs import *
from .train import train


def main():
    files = [*INPUT_DIR.glob("*.parquet")]

    dataset = FreeFlightDataset(files)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / f"{int(time())}_weights.pth"

    train(dataset, output_path)


if __name__ == "__main__":
    main()
