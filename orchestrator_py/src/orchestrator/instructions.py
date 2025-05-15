from dataclasses import dataclass, field


class Serializable:

    def toJSON(self):
        data = self.__dict__.copy()
        name = data.pop("_name", self.__class__.__name__)

        match len(data):
            case 0:
                return {name: None}

            case 1:
                [value] = data.values()
                value = value.toJSON() if isinstance(value, Serializable) else value
                return {name: value}

            case _:
                return data


@dataclass
class Point(Serializable):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0


@dataclass
class Joint(Serializable):
    joint: list[float] = field(default_factory=lambda: 6 * [0.0])

    def toJSON(self):
        return {"Joint": self.joint}


@dataclass
class Profile(Serializable):
    translation_limit: float = field(default=10000.0, metadata={"ge": 0})
    rotation_limit: float = field(default=10000.0, metadata={"ge": 0})
    acceleration_scale: int = field(default=100, metadata={"ge": 0, "le": 100})
    velocity_scale: int = field(default=100, metadata={"ge": 0, "le": 100})
    deceleration_scale: int = field(default=100, metadata={"ge": 0, "le": 100})


@dataclass
class MotionDirect(Serializable):
    point: Point
    _name: str = field(default="Direct")


@dataclass
class MotionJoint(Serializable):
    joint: Joint
    _name: str = field(default="Joint")


@dataclass
class MotionLinear(Serializable):
    point: Point
    _name: str = field(default="Linear")


Motion = MotionDirect | MotionJoint | MotionLinear


@dataclass
class SetBias(Serializable):
    pass


@dataclass
class LoadCell(Serializable):
    type: SetBias


# == Robot Arm == #


@dataclass
class Move(Serializable):
    motion: Motion


@dataclass
class SetOffset(Serializable):
    point: Point


@dataclass
class SetProfile(Serializable):
    profile: Profile


@dataclass
class WaitSettled(Serializable):
    pass


@dataclass
class RobotArm(Serializable):
    type: Move | SetOffset | SetProfile | WaitSettled


# == Wind Shape == #


@dataclass
class EnablePower(Serializable):
    enabled: bool


@dataclass
class ReleaseControl(Serializable):
    pass


@dataclass
class RequestControl(Serializable):
    pass


@dataclass
class SetWindSpeed(Serializable):
    speed: int = 0


@dataclass
class WindShape(Serializable):
    type: EnablePower | ReleaseControl | RequestControl | SetWindSpeed


@dataclass
class Sleep:
    duration: float

    def toJSON(self):
        secs = int(self.duration)

        return {
            "Sleep": {
                "secs": secs,
                "nanos": int((self.duration - secs) * 1e9),
            }
        }


Instruction = LoadCell | RobotArm | WindShape | Sleep
