from dataclasses import replace

from .instructions import Point, Profile


BASE_POINT = Point(x=50.0, y=50.0, z=300.0)

WORKING_POINT = Point(x=1400.0, y=50.0, z=300.0)
WORKING_CONFIG = [3, 2, 0]

ZERO_OFFSET = Point()
LOAD_CELL_OFFSET = Point(x=660.0)
DRONE_OFFSET = Point(x=660.0, z=300.0)

DRONE_HEIGHT: float = 320.0

SLOW_PROFILE = Profile(
    acceleration_scale=30,
    deceleration_scale=30,
    translation_limit=250,
    rotation_limit=90,
)

FAST_PROFILE = Profile(
    acceleration_scale=70,
    deceleration_scale=70,
    translation_limit=1000,
    rotation_limit=180,
)


def create_profile(
    translation_limit: float = 1000, rotation_limit: float = 360
) -> Profile:
    return replace(
        FAST_PROFILE, translation_limit=translation_limit, rotation_limit=rotation_limit
    )


def deg_to_rad(deg: float) -> float:
    return deg * (3.141592653589793 / 180.0)


def rad_to_deg(rad: float) -> float:
    return rad * (180.0 / 3.141592653589793)
