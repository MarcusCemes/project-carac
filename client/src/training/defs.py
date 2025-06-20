from pathlib import Path


from pipeline.dataframe import Columns

# Network configuration
MLP_HIDDEN_LAYERS = [128, 128, 64]  # List of hidden layer sizes for the MLP
LSTM_NUM_LAYERS = 2  # Number of recurrent layers in the LSTM
LSTM_HIDDEN_SIZE = 64  # Number of features in the hidden state of the LSTM
LSTM_SEQUENCE_LENGTH = 20  # Number of time steps to consider in LSTM

# Training configuration
LEARNING_RATE = 0.001
BATCH_SIZE = 64
EPOCHS = 50
VAL_SPLIT = 0.2


MODULE_PATH = Path(__file__).parent
DATA_DIR = (MODULE_PATH / "../../../data").resolve()
INPUT_DIR = DATA_DIR / "processed"
OUTPUT_DIR = DATA_DIR / "checkpoints"

FIG_SIZE = (12, 8)

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

MODEL_COLUMNS = [
    *Columns.BodyForceModel,
    *Columns.BodyMomentModel,
]
