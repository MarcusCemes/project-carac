from pathlib import Path

MODULE_PATH = Path(__file__).parent

# == Parameters == #

LOADED_DATAFRAMES = MODULE_PATH / "data" / "input" / "loaded"
UNLOADED_DATAFRAMES = MODULE_PATH / "data" / "input" / "unloaded"
OUTPUT_DIR = MODULE_PATH / "data" / "output"
LOG_FILE = "pipeline.log"

# == Definitions == #

Vec3 = tuple[float, float, float]
EULER_SEQ = "xyz"  # Extrinsic XYZ Euler angles

COM_OFFSET_M: Vec3 = (-1.05960e-03, -1.12684e-02, 1.85208e-01)

TIME_COL = "time"

WF_X = "robot/x"
WF_Y = "robot/y"
WF_Z = "robot/z"
WF_POSITION = [WF_X, WF_Y, WF_Z]

WF_RX = "robot/roll"
WF_RY = "robot/pitch"
WF_RZ = "robot/yaw"
WF_ATTITUDE = [WF_RX, WF_RY, WF_RZ]

LF_FORCE = ["load/fx", "load/fy", "load/fz"]
LF_MOMENT = ["load/mx", "load/my", "load/mz"]

DF_FORCE = ["fx", "fy", "fz"]
DF_MOMENT = ["mx", "my", "mz"]

DF_PQR = ["p", "q", "r"]
DF_UVW = ["u", "v", "w"]
DF_AERO_FORCES = ["drag", "side_force", "lift"]
DF_AERO_ANGLES = ["alpha", "beta"]

WIND_COL = "wind/speed"
WIND_VEL_LUT = {
    0: 0,
    0.3: 4.1,
    0.5: 5.8,
}
