from enum import EnumDict

from pandas import DataFrame

ROT_ANGLE_SEQ = "xyz"  # Extrinsic XYZ Euler angles


class Columns(EnumDict):
    Time = "time"

    WorldPosition = ["robot/x", "robot/y", "robot/z"]
    WorldRotation = ["robot/roll", "robot/pitch", "robot/yaw"]
    Attitude = ["qx", "qy", "qz", "qw"]

    LoadForce = ["load/fx", "load/fy", "load/fz"]
    LoadMoment = ["load/mx", "load/my", "load/mz"]

    Wind = "wind/speed"

    BodyForce = ["fx", "fy", "fz"]
    BodyMoment = ["mx", "my", "mz"]
    BodyForceModel = ["fx_model", "fy_model", "fz_model"]
    BodyMomentModel = ["mx_model", "my_model", "mz_model"]

    DroneActuators = [
        "drone/motor",
        "drone/left_wing",
        "drone/right_wing",
        "drone/elevator",
        "drone/rudder",
    ]

    AeroAngularVelocity = ["p", "q", "r"]
    AeroVelocity = ["u", "v", "w"]
    AeroAngles = ["alpha", "beta"]
    AeroForces = ["drag", "side_force", "lift"]
    AeroForcesModel = ["drag_model", "side_force_model", "lift_model"]

    ForcePrediction = ["fx_pred", "fy_pred", "fz_pred"]
    MomentPrediction = ["mx_pred", "my_pred", "mz_pred"]


def has_columns(df: DataFrame, columns: list[str]) -> bool:
    return all(col in df.columns for col in columns)
