from .config import WindSpeedLut


def lookup_wind(speed: float) -> float:
    try:
        return WindSpeedLut[speed]

    except KeyError:
        for key, value in WindSpeedLut.items():
            if abs(key - speed) < 1e-6:
                return value

        raise ValueError(f"Wind speed {speed} not found in LUT.")
