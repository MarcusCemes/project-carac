from pathlib import Path

from carac.helpers import Vec3

MODULE_PATH = Path(__file__).parent

# == Parameters == #

ANALYTICAL_MODELLING = False
CORRECT_L_SWEEP = True

# Paths
INPUT_PATH = MODULE_PATH / "data" / "input"
LOADED_DATAFRAMES = INPUT_PATH / "loaded"
UNLOADED_DATAFRAMES = INPUT_PATH / "unloaded"
FREE_FLIGHT_DATAFRAMES = INPUT_PATH / "free-flight"

OUTPUT_DIR = MODULE_PATH / "data" / "output"
LOG_FILE = "pipeline.log"


# == Configuration == #


COM_OFFSET_M: Vec3 = (-1.05960e-03, -1.12684e-02, 1.85208e-01)

WindSpeedLut = {
    0: 0,
    0.3: 4.1,
    0.5: 5.8,
}
