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

                try:
                    return {name: value.toJSON()}

                except Exception:
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

    def add(self, rhs: "Point"):
        return Point(
            x=self.x + rhs.x,
            y=self.y + rhs.y,
            z=self.z + rhs.z,
            rx=self.rx + rhs.rx,
            ry=self.ry + rhs.ry,
            rz=self.rz + rhs.rz,
        )


@dataclass
class Joint(Serializable):
    joint: list[float] = field(default_factory=lambda: 6 * [0.0])

    def toJSON(self):
        return {"Joint": self.joint}


class MotionKind(Serializable, IntEnum):
    Linear = 0
    Direct = 1
    Joint = 2
    Circular = 3

    def toJSON(self):
        return self.value


@dataclass
class Blending(Serializable):
    kind: int = 0
    leave: int = 50
    reach: int = 50


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


@dataclass
class MotionCircular(Serializable):
    intermediate: Point
    point: Point

    def toJSON(self) -> Any:
        return {"Circular": [self.intermediate.toJSON(), self.point.toJSON()]}


Motion = MotionDirect | MotionJoint | MotionLinear | MotionCircular


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
class SetBlending(Serializable):
    blending: Blending


@dataclass
class SetConfig(Serializable):
    config: list[int]


@dataclass
class SetOffset(Serializable):
    point: Point


@dataclass
class SetPowered(Serializable):
    powered: bool


@dataclass
class SetProfile(Serializable):
    profile: Profile


@dataclass
class SetReportInterval(Serializable):
    interval_s: float = 0.1


@dataclass
class WaitSettled(Serializable):
    pass


@dataclass
class GoHome(Serializable):
    type: MotionKind = MotionKind.Joint

    def toJSON(self):
        return {"GoHome": self.type.toJSON()}


@dataclass
class RobotArm(Serializable):
    type: (
        Move
        | GoHome
        | SetBlending
        | SetConfig
        | SetOffset
        | SetPowered
        | SetProfile
        | SetReportInterval
        | WaitSettled
    )


# == Wind Shape == #


@dataclass
class EnablePower(Serializable):
    enabled: bool


@dataclass
class SetWindSpeed(Serializable):
    speed: int = 0


@dataclass
class WindShape(Serializable):
    type: EnablePower | SetWindSpeed


@dataclass
class BiasAll(Serializable):
    pass


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


Instruction = LoadCell | RobotArm | WindShape | BiasAll | Sleep | Reset
