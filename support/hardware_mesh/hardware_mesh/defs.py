from enum import StrEnum


DEFAULT_TIMEOUT = 3.0
DEFAULT_QOS = 16


class Topics(StrEnum):
    LoadCellData = "load_cell_data"
    RobotArmMoving = "robot_arm_moving"
    RobotArmPose = "robot_arm_pose"


class Services(StrEnum):
    LoadCellSetBias = "load_cell_set_bias"
    LoadCellStream = "load_cell_stream"
    RobotArmConfig = "robot_arm_config"
    RobotArmTool = "robot_arm_tool"
    WindShape = "wind_shape"
