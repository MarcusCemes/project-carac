from concurrent.futures import ProcessPoolExecutor
from io import TextIOWrapper
from typing import Sequence

from pandas import DataFrame, read_parquet
from rich.progress import Progress

from .augment import augment_dataframe
from .dataset import ExperimentComposition
from .defs import *


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

            for composition in pool.map(process_composition, jobs):
                if not warm:
                    progress.update(
                        task,
                        total=len(compositions),
                        description=f"Processing [b]{name}[/]",
                    )

                    warm = True

                log_composition(composition, log)
                progress.advance(task, 1)


def process_composition(
    args: tuple[Path, ExperimentComposition],
) -> ExperimentComposition:
    (output_path, composition) = args

    df = combine_composition(composition)
    augment_dataframe(df)

    parquet_path = output_path / composition.experiment.name
    df.to_parquet(parquet_path)

    return composition


def combine_composition(composition: ExperimentComposition) -> DataFrame:
    """Combines the positive and negative DataFrames in a composition."""

    experiment = read_parquet(composition.experiment)

    for p in map(read_parquet, composition.positive):
        experiment[LF_FORCE] += p[LF_FORCE]
        experiment[LF_MOMENT] += p[LF_MOMENT]

    for n in map(read_parquet, composition.negative):
        experiment[LF_FORCE] -= n[LF_FORCE]
        experiment[LF_MOMENT] -= n[LF_MOMENT]

    return experiment


def log_composition(composition: ExperimentComposition, log: TextIOWrapper) -> None:
    """Logs the details of an ExperimentComposition to a file."""

    padding = max(
        len(c.stem)
        for c in (composition.experiment, *composition.positive, *composition.negative)
    )

    log_file(composition.experiment, " ", padding, log)

    for positive in composition.positive:
        log_file(positive, "+", padding, log)

    for negative in composition.negative:
        log_file(negative, "-", padding, log)

    log.write("\n")


def log_file(file: Path, prefix: str, padding: int, log: TextIOWrapper) -> None:
    log.write(f" {prefix} {file.stem.ljust(padding)} ({file.parent.name})\n")
