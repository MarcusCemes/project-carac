from pathlib import Path


# == Features/Corrections == #

ANALYTICAL_MODELLING = True

USE_POOL: bool = True
FILTER: list[str] = []


# == Paths == #

DATA_PATH = (Path(__file__).parent / "../../../data").resolve()
INPUT_PATH = DATA_PATH / "unprocessed"
OUTPUT_PATH = DATA_PATH / "processed"
