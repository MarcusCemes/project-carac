from .datasets import FreeFlight2Dataset
from .defs import *
from .train import train_and_save_models


def main():
    dataset = FreeFlight2Dataset()
    train_and_save_models(dataset)


if __name__ == "__main__":
    main()
