from concurrent.futures import ProcessPoolExecutor
from io import TextIOWrapper
from typing import Any

from rich import print as rprint
from rich.progress import Progress
from rich.status import Status

from .config import *
from .data.datasets import LOADERS, Loader
from .data.utils import ExperimentBundle
from .utils import *


def main() -> None:
    rprint("[b]CARAC Pipeline[/] - Automated pipeline for processing CARAC datasets\n")

    if USE_POOL:
        with ProcessPoolExecutor() as pool:
            for loader in LOADERS:
                process_dataset(loader, pool.map)

    else:
        for loader in LOADERS:
            process_dataset(loader, map)

    rprint("[green b]All datasets processed successfully[/]")


def process_dataset(
    loader: Loader,
    map_fn: Any,
) -> None:
    """Processes a dataset of ExperimentComposition objects."""

    with Status("Preparing worker pool"):
        from .worker import process_bundle

    output_path = OUTPUT_PATH / loader.name
    output_path.mkdir(parents=True, exist_ok=True)

    try:
        with open(output_path / "pipeline.log", "w") as log:
            with Progress(transient=True) as progress:
                task = progress.add_task("Creating experiment bundles", total=None)

                bundles = loader.load_bundles()

                jobs = (
                    (bundle, output_path, loader)
                    for bundle in bundles
                    if (
                        not FILTER
                        or any(bundle.primary.path.name.startswith(f) for f in FILTER)
                    )
                )

                progress.update(
                    task,
                    total=len(bundles),
                    description=f"Processing [b]{loader.name}[/] dataset",
                )

                for bundle in map_fn(process_bundle, jobs):
                    append_composition_to_log(bundle, log)
                    progress.advance(task, 1)

        rprint(
            f"Processed [b]{loader.name}[/] dataset with {len(bundles)} compositions"
        )

    except Exception as e:
        rprint(f"[red]Error processing {loader.name} dataset")
        raise


def append_composition_to_log(
    bundle: ExperimentBundle,
    file: TextIOWrapper,
) -> None:
    """Logs the details of an ExperimentComposition to a file."""

    padding = max(
        len(c.path.stem) for c in (bundle.primary, *bundle.positive, *bundle.negative)
    )

    log_file_with_prefix(bundle.primary.path, " ", padding, file)

    for positive in bundle.positive:
        log_file_with_prefix(positive.path, "+", padding, file)

    for negative in bundle.negative:
        log_file_with_prefix(negative.path, "-", padding, file)

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
