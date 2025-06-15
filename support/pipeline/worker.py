from pandas import DataFrame, read_parquet

from .process import process_dataframe
from .dataset import ExperimentComposition
from .defs import *


def process_composition(
    args: tuple[Path, ExperimentComposition],
) -> ExperimentComposition:
    (output_path, composition) = args

    df = combine_composition(composition)
    process_dataframe(df)

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
