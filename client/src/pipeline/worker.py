from pathlib import Path
from pandas import DataFrame, read_parquet

from .data.datasets import Loader
from .data.utils import ExperimentBundle
from .dataframe import *
from .process import process_dataframe


def process_bundle(args: tuple[ExperimentBundle, Path, Loader]) -> ExperimentBundle:
    (bundle, output, loader) = args

    df = load_and_combine_bundle(bundle)

    loader.preprocess(df, bundle.primary)
    process_dataframe(df, loader)
    loader.postprocess(df, bundle.primary)

    df.to_parquet(output / bundle.primary.path.name)
    return bundle


def load_and_combine_bundle(bundle: ExperimentBundle) -> DataFrame:
    primary = read_parquet(bundle.primary.path)

    for p in bundle.positive:
        df = p.read()
        primary[Columns.LoadForce] += df[Columns.LoadForce]
        primary[Columns.LoadMoment] += df[Columns.LoadMoment]

    for n in bundle.negative:
        df = n.read()
        primary[Columns.LoadForce] -= df[Columns.LoadForce]
        primary[Columns.LoadMoment] -= df[Columns.LoadMoment]

    return primary
