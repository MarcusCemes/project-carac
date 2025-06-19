EPSILON = 1e-6


class WindLut:

    def __init__(self, lut: dict[float, float]):
        if any(k < 0 or 1 < k for k in lut):
            raise ValueError("Invalid LUT")

        self._lut = lut

    def lookup(self, speed: float):
        if speed == 0:
            return 0

        if not 0 < speed <= 1:
            raise ValueError(f"Invalid wind speed: {speed}")

        for key, value in self._lut.items():
            if abs(key - speed) < EPSILON:
                return value

        raise ValueError(f"Wind speed {speed} not in LUT")
