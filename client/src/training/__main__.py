from .datasets import AxisDataset
from .defs import *
from .train import train_and_save_models


def main():
    dataset = AxisDataset()
    train_and_save_models(dataset)


if __name__ == "__main__":
    main()
