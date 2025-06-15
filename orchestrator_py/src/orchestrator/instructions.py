from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Sequence


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

    def toJSON(self) -> Any:
        return {
            "position": [self.x, self.y, self.z],
            "orientation": [self.rx, self.ry, self.rz],
        }


@dataclass
class Joint(Serializable):
    joint: Sequence[float] = field(default_factory=lambda: 6 * [0.0])

    def toJSON(self):
        return self.joint


class MotionKind(Serializable, IntEnum):
    Linear = 0
    Direct = 1
    Joint = 2
    Circular = 3

    def toJSON(self):
        return self.name


class BlendingKind(Serializable, IntEnum):
    Off = 0
    Joint = 1
    Cartesian = 2

    def toJSON(self):
        return self.name


@dataclass
class Blending(Serializable):
    kind: BlendingKind = field(default=BlendingKind.Off)
    leave: int = 50
    reach: int = 50

    def toJSON(self) -> Any:
        return {
            "kind": self.kind.toJSON(),
            "leave": self.leave,
            "reach": self.reach,
        }


@dataclass
class ProfileLimit(Serializable):
    translation: float = 10000.0
    rotation: float = 10000.0


@dataclass
class ProfileScale(Serializable):
    acceleration: int = 100
    velocity: int = 100
    deceleration: int = 100


@dataclass
class Profile(Serializable):
    limit: ProfileLimit = field(default_factory=ProfileLimit)
    scale: ProfileScale = field(default_factory=ProfileScale)

    def toJSON(self) -> Any:
        return {
            "limit": self.limit.toJSON(),
            "scale": self.scale.toJSON(),
        }


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


# == Device == #


@dataclass
class Device(Serializable):
    name: str
    command: Sequence[float]

    def toJSON(self) -> Any:
        return {"Device": [self.name, self.command]}


# == Load == #


@dataclass
class SetBias(Serializable):
    pass


@dataclass
class Load(Serializable):
    type: SetBias


# == Robot Arm == #


@dataclass
class Move(Serializable):
    motion: Motion


@dataclass
class SetBlending(Serializable):
    blending: Blending


class ConfigKind(Serializable, IntEnum):
    Free = 0
    Same = 1
    RightyPositive = 2
    LeftyNegative = 3

    def toJSON(self):
        return self.name


@dataclass
class Config(Serializable):
    shoulder: ConfigKind = field(default=ConfigKind.Free)
    elbow: ConfigKind = field(default=ConfigKind.Free)
    wrist: ConfigKind = field(default=ConfigKind.Free)

    def toJSON(self) -> Any:
        return {
            "shoulder": self.shoulder.toJSON(),
            "elbow": self.elbow.toJSON(),
            "wrist": self.wrist.toJSON(),
        }


@dataclass
class SetConfig(Serializable):
    config: Config


@dataclass
class SetToolOffset(Serializable):
    point: Point


@dataclass
class ResetMoveId(Serializable):
    pass


@dataclass
class SetPowered(Serializable):
    powered: bool


@dataclass
class SetProfile(Serializable):
    profile: Profile


@dataclass
class SetFrequency(Serializable):
    interval_s: float = 0.1


@dataclass
class SetReporting(Serializable):
    enabled: bool = True


@dataclass
class WaitSettled(Serializable):
    pass


@dataclass
class WaitProgress(Serializable):
    progress: float


@dataclass
class ReturnHome(Serializable):
    type: MotionKind = MotionKind.Joint

    def toJSON(self):
        return {"ReturnHome": self.type.toJSON()}


@dataclass
class Robot(Serializable):
    type: (
        Move
        | ReturnHome
        | SetBlending
        | SetConfig
        | SetFrequency
        | SetPowered
        | SetProfile
        | SetReporting
        | SetToolOffset
        | ResetMoveId
        | WaitSettled
        | WaitProgress
    )


# == Wind Shape == #


class SetFanSpeed(Serializable):
    speed: float = 0

    def __init__(self, speed: float = 0.0):
        if speed > 1:
            raise ValueError("Fan speed too high! Must be in [0, 1].")

        self.speed = speed


@dataclass
class Wind(Serializable):
    type: SetPowered | SetFanSpeed | WaitSettled


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


Instruction = Device | Load | Robot | Wind | BiasAll | Sleep | Reset
