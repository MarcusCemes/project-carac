from dataclasses import dataclass
from pathlib import Path
from itertools import cycle
from typing import TypeVar, List

from .defs import *

T = TypeVar("T")


@dataclass
class CalibratedExperiment:
    actual: Path
    calibration: Path


@dataclass
class ExperimentComposition:
    experiment: Path
    positive: list[Path]
    negative: list[Path]


def load_datasets():
    # Load the sessions and their respective coupled and decoupled experiments.
    session_loaded = load_experiments(LOADED_DATAFRAMES)
    loaded_decoupled = session_loaded[0:486]
    loaded_coupled = session_loaded[486:558]

    session_unloaded = load_experiments(UNLOADED_DATAFRAMES)
    unloaded_decoupled = session_unloaded[0:18]
    unloaded_coupled = session_unloaded[18:26]

    # --- Decoupled Runs ---#

    # The unloaded data is a single block of 18 files (6 maneuvers x 3 wind speeds)
    unloaded_decoupled_calibrated = pair_with_calibration(unloaded_decoupled, stride=6)

    # The loaded data is composed of 27 blocks of 18 files each.
    loaded_decoupled_chunks = chunk_list(loaded_decoupled, chunk_size=18)

    decoupled_compositions: list[ExperimentComposition] = []

    for loaded_chunk in loaded_decoupled_chunks:
        loaded_chunk_calibrated = pair_with_calibration(loaded_chunk, stride=6)

        # Compose this specific loaded chunk with the (reused) unloaded data
        compositions = compose_experiment_loads(
            unloaded_decoupled_calibrated,
            loaded_chunk_calibrated,
        )

        decoupled_compositions.extend(compositions)

    # --- Coupled Runs --- #

    # Unloaded: 8 files (4 maneuvers x 2 wind speeds)
    unloaded_coupled_calibrated = pair_with_calibration(unloaded_coupled, stride=4)

    # Loaded: 72 files (36 maneuvers x 2 wind speeds). This is a single block.
    loaded_coupled_calibrated = pair_with_calibration(loaded_coupled, stride=36)

    coupled_compositions = compose_experiment_loads(
        unloaded_coupled_calibrated,
        loaded_coupled_calibrated,
    )

    assert len(decoupled_compositions) == 324
    assert len(coupled_compositions) == 36

    return (decoupled_compositions, coupled_compositions)


def load_experiments(path: Path) -> list[Path]:
    """Read a list of experiment parquet's from a given path."""

    experiments = [
        file
        for file in path.iterdir()
        if file.name[0:4].isnumeric() and file.suffix == ".parquet"
    ]

    if not experiments:
        raise ValueError(
            f"No experiments found in {path}. Ensure the path is correct and contains parquet files."
        )

    experiments.sort()

    return experiments


def chunk_list(data: List[T], chunk_size: int) -> List[List[T]]:
    """Splits a list into smaller lists of a specified size."""

    if len(data) % chunk_size != 0:
        raise ValueError(
            f"List length {len(data)} is not divisible by chunk size {chunk_size}"
        )

    return [data[i : i + chunk_size] for i in range(0, len(data), chunk_size)]


def pair_with_calibration(
    experiments: list[Path],
    stride: int,
) -> list[CalibratedExperiment]:
    """
    Pairs up `stride` number of calibration files (no wind), followed by
    `N*stride` actual runs (wind). This assumes that the first `stride` files
    are the calibration files that are cycled and paired with the following
    actual runs (multiple of `stride`).

    **The ordering is vital for this to work**.
    """

    calibrations = experiments[:stride]
    actual_runs = experiments[stride:]

    return [
        CalibratedExperiment(actual=actual, calibration=calibration)
        for calibration, actual in zip(cycle(calibrations), actual_runs)
    ]


def compose_experiment_loads(
    unloaded_calibrated: list[CalibratedExperiment],
    loaded_calibrated: list[CalibratedExperiment],
) -> list[ExperimentComposition]:
    """
    Composes calibrated pairs of loaded and unloaded experiments into a
    quartet of contributions, following the equations to isolate the drone's
    aerodynamic forces:

    `(load_actual - load_calibration) - (unload_actual - unload_calibration)`
    """

    return [
        ExperimentComposition(
            experiment=loaded.actual,
            positive=[unloaded.calibration],
            negative=[loaded.calibration, unloaded.actual],
        )
        for unloaded, loaded in zip(cycle(unloaded_calibrated), loaded_calibrated)
    ]


def pretty_print_composition(composition: ExperimentComposition):
    for path in composition.positive:
        print(f" +{path.name} ({path.parent.parent.name})")

    for path in composition.negative:
        print(f" -{path.name} ({path.parent.parent.name})")

    print()
