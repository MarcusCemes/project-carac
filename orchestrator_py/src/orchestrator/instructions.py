from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any


class Serializable:

    def toJSON(self) -> Any:
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


class MotionKind(IntEnum, Serializable):
    Linear = 0
    Direct = 1
    Joint = 2

    def toJSON(self):
        self.value


@dataclass
class Profile(Serializable):
    translation_limit: float = 10000.0
    rotation_limit: float = 10000.0
    acceleration_scale: int = 100
    velocity_scale: int = 100
    deceleration_scale: int = 100


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
class SetConfig(Serializable):
    config: list[int]


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
class GoHome(Serializable):
    type: MotionKind = MotionKind.Joint


@dataclass
class RobotArm(Serializable):
    type: Move | GoHome | SetConfig | SetOffset | SetProfile | WaitSettled


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


@dataclass
class Reset(Serializable):
    pass


Instruction = LoadCell | RobotArm | WindShape | Sleep | Reset
