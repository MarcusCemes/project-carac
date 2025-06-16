from .defs import *


def lookup_wind(speed: float) -> float:
    try:
        return WindSpeedLut[speed]

    except KeyError:
        raise ValueError(f"Wind speed {speed} not found in LUT.")
