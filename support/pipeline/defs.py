from pathlib import Path

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


# == Types == #

Vec3 = tuple[float, float, float]


# == Configuration == #

EULER_SEQ = "xyz"  # Extrinsic XYZ Euler angles

COM_OFFSET_M: Vec3 = (-1.05960e-03, -1.12684e-02, 1.85208e-01)

TIME_COL = "time"

WF_POSITION = ["robot/x", "robot/y", "robot/z"]
WF_ATTITUDE = ["robot/roll", "robot/pitch", "robot/yaw"]
WF_QUATERNION = ["qx", "qy", "qz", "qw"]

LF_FORCE = ["load/fx", "load/fy", "load/fz"]
LF_MOMENT = ["load/mx", "load/my", "load/mz"]

DF_FORCE = ["fx", "fy", "fz"]
DF_MOMENT = ["mx", "my", "mz"]

DF_FORCE_MODEL = ["fx_model", "fy_model", "fz_model"]
DF_MOMENT_MODEL = ["mx_model", "my_model", "mz_model"]

DRONE_ACT = [
    "drone/motor",
    "drone/left_wing",
    "drone/right_wing",
    "drone/elevator",
    "drone/rudder",
]

DF_PQR = ["p", "q", "r"]
DF_UVW = ["u", "v", "w"]
DF_AERO_ANGLES = ["alpha", "beta"]
DF_AERO_FORCES = ["drag", "side_force", "lift"]
DF_AERO_FORCES_MODEL = ["drag_model", "side_force_model", "lift_model"]

WIND_COL = "wind/speed"
WIND_VEL_LUT = {
    0: 0,
    0.3: 4.1,
    0.5: 5.8,
}
