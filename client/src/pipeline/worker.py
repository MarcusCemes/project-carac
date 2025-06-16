from pandas import DataFrame, read_parquet

from .dataset import ExperimentComposition
from .dataframe import *
from .defs import *
from .process import process_dataframe


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
        experiment[Columns.LoadForce] += p[Columns.LoadForce]
        experiment[Columns.LoadMoment] += p[Columns.LoadMoment]

    for n in map(read_parquet, composition.negative):
        experiment[Columns.LoadForce] -= n[Columns.LoadForce]
        experiment[Columns.LoadMoment] -= n[Columns.LoadMoment]

    return experiment
