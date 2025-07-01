from pathlib import Path


from pipeline.dataframe import Columns

# Network configuration
MLP_HIDDEN_LAYERS = [128, 64, 64]  # List of hidden layer sizes for the MLP
LSTM_NUM_LAYERS = 2  # Number of recurrent layers in the LSTM
LSTM_HIDDEN_SIZE = 32  # Number of features in the hidden state of the LSTM
LSTM_SEQUENCE_LENGTH = 5  # Number of time steps to consider in LSTM

# MLP_HIDDEN_LAYERS = [32, 16]  # List of hidden layer sizes for the MLP
# LSTM_NUM_LAYERS = 1  # Number of recurrent layers in the LSTM
# LSTM_HIDDEN_SIZE = 16  # Number of features in the hidden state of the LSTM
# LSTM_SEQUENCE_LENGTH = 5  # Number of time steps to consider in LSTM

# MLP_HIDDEN_LAYERS = [256, 128, 64]  # List of hidden layer sizes for the MLP
# LSTM_NUM_LAYERS = 3  # Number of recurrent layers in the LSTM
# LSTM_HIDDEN_SIZE = 64  # Number of features in the hidden state of the LSTM
# LSTM_SEQUENCE_LENGTH = 10  # Number of time steps to consider in LSTM

MLP_MULTI_LAYER_SIZES = [
    [1],
    [8],
    [2, 2],
    [16, 8],
    [32, 16],
    [128, 128, 64],
    [256, 128, 64, 16],
]


# Training configuration
LEARNING_RATE = 0.001
DROPOUT_RATE = 0.2
BATCH_SIZE = 256
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
