from concurrent.futures import ProcessPoolExecutor
from io import TextIOWrapper
from typing import Any, Sequence

from rich import print as rprint
from rich.progress import Progress

from .config import *
from .dataset import load_datasets, ExperimentComposition
from .utils import *


def main() -> None:
    rprint("[b]CARAC Pipeline[/] - Automated pipeline for processing CARAC datasets\n")

    datasets = load_datasets()

    if USE_POOL:
        with ProcessPoolExecutor() as pool:
            for dataset, name in datasets:
                process_dataset(name, dataset, pool.map)

    else:
        for dataset, name in datasets:
            process_dataset(name, dataset, map)

    rprint("[green b]All datasets processed successfully[/]")


def process_dataset(
    name: str,
    compositions: Sequence[ExperimentComposition],
    map_fn: Any,
) -> None:
    """Processes a dataset of ExperimentComposition objects."""

    output_path = DATA_PATH / "output" / name
    output_path.mkdir(parents=True, exist_ok=True)

    warm = False

    with open(output_path / "pipeline.log", "w") as log:
        with Progress(transient=True) as progress:
            task = progress.add_task("Warming up the pool...", total=None)
            jobs = (
                (output_path, composition)
                for composition in compositions
                if not FILTER
                or any(composition.experiment.name.startswith(f) for f in FILTER)
            )

            from .worker import process_composition

            for composition in map_fn(process_composition, jobs):
                if not warm:
                    progress.update(
                        task,
                        total=len(compositions),
                        description=f"Processing [b]{name}[/]",
                    )

                    warm = True

                append_composition_to_log(composition, log)
                progress.advance(task, 1)

    rprint(f"Processed [b]{name}[/] dataset with {len(compositions)} compositions")


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
