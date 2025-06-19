from dataclasses import dataclass, field
from itertools import cycle
from pathlib import Path
from re import compile as re_compile
from typing import Sequence, TypeVar

import numpy as np
from pandas import DataFrame, read_parquet

from pipeline.dataframe import Columns


T = TypeVar("T")


@dataclass
class ParsedName:
    id: int | None
    name: str
    parameters: dict[str, float]


@dataclass
class LoadedExperiment:
    path: Path
    _parsed: ParsedName | None = None

    def parsed(self) -> ParsedName:
        if self._parsed is None:
            self._parsed = parse_experiment_name(self.path.name)

        return self._parsed

    def read(self) -> DataFrame:
        return read_parquet(self.path)


@dataclass
class ExperimentBundle:
    primary: LoadedExperiment
    positive: list[LoadedExperiment] = field(default_factory=list)
    negative: list[LoadedExperiment] = field(default_factory=list)

    def __sub__(self, other: "ExperimentBundle") -> "ExperimentBundle":
        return ExperimentBundle(
            self.primary,
            self.positive + other.negative,
            [other.primary] + other.positive + self.negative,
        )


MAIN_PATTERN = re_compile(r"^(?:(\d+)_)?([\w-]+?)_(.+)\.parquet$")
PARAM_PATTERN = re_compile(r"([a-zA-Z])(-?\d*\.?\d+)")


def parse_experiment_name(filename: str) -> ParsedName:
    match = MAIN_PATTERN.match(filename)

    if not match:
        raise ValueError(f"Filename '{filename}' does not match expected format.")

    id_str, name, params_str = match.groups()
    file_id = int(id_str) if id_str else None

    params_dict = {}

    for key, value_str in PARAM_PATTERN.findall(params_str):
        params_dict[key] = float(value_str)

    return ParsedName(file_id, name, params_dict)


def is_associated_experiment(experiment: ParsedName, candidate: ParsedName) -> bool:
    return all(
        candidate.parameters[k] == v
        for (k, v) in experiment.parameters.items()
        if k != "w"
    )


def chunk_list(data: list[T], chunk_size: int) -> list[list[T]]:
    """Splits a list into smaller lists of a specified size."""

    if len(data) % chunk_size != 0:
        raise ValueError(
            f"List length {len(data)} is not divisible by chunk size {chunk_size}"
        )

    return [data[i : i + chunk_size] for i in range(0, len(data), chunk_size)]


def combine_calibrated_bundles(
    positive_bundles: Sequence[ExperimentBundle],
    negative_bundles: Sequence[ExperimentBundle],
) -> list[ExperimentBundle]:
    assert len(positive_bundles) % len(negative_bundles) == 0
    return [p - n for (p, n) in zip(positive_bundles, cycle(negative_bundles))]


def combine_strided_bundles(
    bundles: list[ExperimentBundle],
    stride: int,
) -> list[ExperimentBundle]:
    """
    Pairs up `stride` number of experiments files (no wind), followed by
    `N*stride` actual experiments (wind). This assumes that the first `stride`
    files are the calibration files that are cycled and paired with the following
    actual runs (multiple of `stride`).

    **The ordering is vital for this to work**.
    """

    return combine_calibrated_bundles(
        positive_bundles=bundles[stride:],
        negative_bundles=bundles[:stride],
    )
