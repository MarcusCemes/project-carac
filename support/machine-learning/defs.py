# Network configuration
HIDDEN_LAYERS = [128, 128, 64]


# Training configuration
LEARNING_RATE = 0.001
BATCH_SIZE = 64
EPOCHS = 50
VAL_SPLIT = 0.2


# Dataset configuration
DATA_DIR = "data"

EULER_ORDER = "XYZ"
ATTITUDE_COLUMNS = ["robot/roll", "robot/pitch", "robot/yaw"]

ATTITUDE_QUAT_COLUMNS = ["qx", "qy", "qz", "qw"]
ANGULAR_VELOCITY_COLUMNS = ["p", "q", "r"]
RELATIVE_VELOCITY_COLUMNS = ["u", "v", "w"]


DRONE_ACTUATOR_COLUMNS = [
    "drone/motor",
    "drone/left_wing",
    "drone/right_wing",
    "drone/elevator",
    "drone/rudder",
]

WIND_COLUMN = "wind/speed"

INPUT_COLUMNS = [
    *ATTITUDE_QUAT_COLUMNS,
    *ANGULAR_VELOCITY_COLUMNS,
    *RELATIVE_VELOCITY_COLUMNS,
    *DRONE_ACTUATOR_COLUMNS,
    WIND_COLUMN,
]

OUTPUT_COLUMNS = [
    "fx",
    "fx",
    "fx",
    "mx",
    "my",
    "mz",
]
