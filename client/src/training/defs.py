from pathlib import Path


from pipeline.dataframe import Columns

# Network configuration
HIDDEN_LAYERS = [128, 128, 64]


# Training configuration
LEARNING_RATE = 0.001
BATCH_SIZE = 64
EPOCHS = 50
VAL_SPLIT = 0.2


MODULE_PATH = Path(__file__).parent
DATA_DIR = (MODULE_PATH / "../../../data-old").resolve()
INPUT_DIR = DATA_DIR / "output/free-flight"
OUTPUT_DIR = DATA_DIR / "checkpoints/free-flight"

INPUT_COLUMNS = [
    *Columns.Attitude,
    *Columns.AeroAngularVelocity,
    *Columns.AeroVelocity,
    *Columns.DroneActuators,
]

OUTPUT_COLUMNS = [
    *Columns.BodyForce,
    *Columns.BodyMoment,
]
