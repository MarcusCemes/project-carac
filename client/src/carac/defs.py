from enum import EnumDict

from .instructions import (
    Blending,
    BlendingKind,
    Config,
    ConfigKind,
    Point,
    Profile,
    ProfileLimit,
    ProfileScale,
)

BASE_POINT = Point(x=50.0, y=50.0, z=300.0)

LOAD_CELL_DISTANCE_X: float = 660.0
DRONE_DISTANCE_Z: float = 320.0


DRONE_NO_ACTUATION = [-1.0, 0.0, 0.0, 0.0, 0.0]


class Configs(EnumDict):
    Free = Config()

    Working = Config(
        ConfigKind.LeftyNegative,
        ConfigKind.RightyPositive,
        ConfigKind.Free,
    )


class Offsets(EnumDict):
    LoadCell = Point(x=LOAD_CELL_DISTANCE_X)
    Drone = Point(x=LOAD_CELL_DISTANCE_X, z=DRONE_DISTANCE_Z)


class Points(EnumDict):
    Base = BASE_POINT
    DroneBase = BASE_POINT.add(Offsets.Drone)
    Working = Point(x=1400.0, y=50.0, z=300.0)
    WorkingPointClose = Point(x=700.0, y=500.0, z=350.0)


class Blends(EnumDict):
    Zero = Blending(BlendingKind.Cartesian, leave=0, reach=0)
    Small = Blending(BlendingKind.Cartesian, leave=50, reach=50)
    Medium = Blending(BlendingKind.Cartesian, leave=150, reach=150)
    Large = Blending(BlendingKind.Cartesian, leave=300, reach=300)


class Profiles(EnumDict):
    Slow = Profile(
        limit=ProfileLimit(translation=250, rotation=90),
        scale=ProfileScale(acceleration=30, deceleration=30),
    )

    Medium = Profile(
        limit=ProfileLimit(translation=500, rotation=120),
        scale=ProfileScale(acceleration=50, deceleration=50),
    )

    Fast = Profile(
        limit=ProfileLimit(translation=1000, rotation=180),
        scale=ProfileScale(acceleration=70, deceleration=70),
    )
