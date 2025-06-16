from enum import EnumDict

ROT_ANGLE_SEQ = "xyz"  # Extrinsic XYZ Euler angles


class Columns(EnumDict):
    Time = "time"

    RobotPos = ["robot/x", "robot/y", "robot/z"]
    RobotRot = ["robot/roll", "robot/pitch", "robot/yaw"]
    Attitude = ["qx", "qy", "qz", "qw"]

    LoadForce = ["load/fx", "load/fy", "load/fz"]
    LoadMoment = ["load/mx", "load/my", "load/mz"]

    Wind = "wind/speed"

    DroneForce = ["fx", "fy", "fz"]
    DroneMoment = ["mx", "my", "mz"]
    DroneForceModel = ["fx_model", "fy_model", "fz_model"]
    DroneMomentModel = ["mx_model", "my_model", "mz_model"]

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
