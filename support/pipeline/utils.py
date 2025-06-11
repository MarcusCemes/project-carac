from .defs import *


def lookup_wind(speed: float) -> float:
    try:
        return WIND_VEL_LUT[speed]

    except KeyError:
        raise ValueError(f"Wind speed {speed} not found in LUT.")
