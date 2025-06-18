from pathlib import Path
from carac.helpers import Vec3


# == Features/Corrections == #

ANALYTICAL_MODELLING = True
CORRECT_L_SWEEP = True


# == Paths == #

DATA_PATH = (Path(__file__).parent / "../../../data-old").resolve()
INPUT_PATH = DATA_PATH / "input"

USE_POOL: bool = True
FILTER: list[str] = []


# == Measurements == #

COM_OFFSET_M: Vec3 = (-1.12684e-02, -1.05960e-03, 1.85208e-01)

WindSpeedLut = {
    0: 0,
    0.3: 4.1,
    0.46: 5.0,
    0.5: 5.8,
}
