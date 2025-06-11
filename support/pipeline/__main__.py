from concurrent.futures import ProcessPoolExecutor

from rich import print as rprint

from .dataset import load_datasets
from .defs import *
from .worker import process_dataset
from .utils import *


def main() -> None:
    rprint("[b]CARAC Pipeline[/] - Automated pipeline for processing CARAC datasets.\n")

    (decoupled, coupled) = load_datasets()

    jobs = [
        (decoupled, "decoupled"),
        (coupled, "coupled"),
    ]

    with ProcessPoolExecutor() as pool:
        for dataset, name in jobs:
            process_dataset(name, dataset, pool)
            rprint(f"Processed [b]{name}[/] dataset with {len(dataset)}")

    rprint("[green b]All datasets processed successfully[/]")


if __name__ == "__main__":
    main()
