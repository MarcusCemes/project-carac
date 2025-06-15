from concurrent.futures import ProcessPoolExecutor
from io import TextIOWrapper
from typing import Sequence

from rich import print as rprint
from rich.progress import Progress

from .dataset import load_datasets, ExperimentComposition
from .defs import *
from .utils import *


def main() -> None:
    rprint("[b]CARAC Pipeline[/] - Automated pipeline for processing CARAC datasets\n")

    (decoupled, coupled, free_flight) = load_datasets()

    jobs = [
        (decoupled, "decoupled"),
        (coupled, "coupled"),
        (free_flight, "free_flight"),
    ]

    with ProcessPoolExecutor() as pool:

        for dataset, name in jobs:
            process_dataset(name, dataset, pool)
            rprint(f"Processed [b]{name}[/] dataset with {len(dataset)}")

    rprint("[green b]All datasets processed successfully[/]")


def process_dataset(
    name: str,
    compositions: Sequence[ExperimentComposition],
    pool: ProcessPoolExecutor,
) -> None:
    """Processes a dataset of ExperimentComposition objects."""

    output_path = Path(OUTPUT_DIR) / name
    output_path.mkdir(parents=True, exist_ok=True)

    warm = False

    with open(output_path / LOG_FILE, "w") as log:
        with Progress(transient=True) as progress:
            task = progress.add_task("Warming up the pool...", total=None)
            jobs = ((output_path, composition) for composition in compositions)

            from .worker import process_composition

            for composition in pool.map(process_composition, jobs):
                if not warm:
                    progress.update(
                        task,
                        total=len(compositions),
                        description=f"Processing [b]{name}[/]",
                    )

                    warm = True

                append_composition_to_log(composition, log)
                progress.advance(task, 1)


def append_composition_to_log(
    composition: ExperimentComposition, file: TextIOWrapper
) -> None:
    """Logs the details of an ExperimentComposition to a file."""

    padding = max(
        len(c.stem)
        for c in (composition.experiment, *composition.positive, *composition.negative)
    )

    log_file_with_prefix(composition.experiment, " ", padding, file)

    for positive in composition.positive:
        log_file_with_prefix(positive, "+", padding, file)

    for negative in composition.negative:
        log_file_with_prefix(negative, "-", padding, file)

    file.write("\n")


def log_file_with_prefix(
    file: Path,
    prefix: str,
    padding: int,
    log: TextIOWrapper,
) -> None:
    log.write(f" {prefix} {file.stem.ljust(padding)} ({file.parent.name})\n")


if __name__ == "__main__":
    main()
