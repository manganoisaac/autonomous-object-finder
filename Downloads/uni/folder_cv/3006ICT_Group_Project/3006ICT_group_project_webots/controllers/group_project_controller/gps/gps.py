from dataclasses import dataclass
from math import floor, pi
from typing import Any


@dataclass(frozen=True)
class GPS:
    gps: Any
    imu: Any
    map: list[list[int]]
    x_min: float = -2.0
    x_max: float = 2.0
    y_min: float = -2.0
    y_max: float = 2.0
    interval: float = 0.1

    def coordinate(self) -> tuple[int, int]:
        x, y, _ = self.gps.getValues()
        x_discrete, y_discrete = floor((x + 2) * 10), floor((y + 2) * 10)
        return x_discrete, y_discrete

    def direction(self) -> tuple[int, float]:
        yaw = self.imu.getRollPitchYaw()[2]
        direction = round(yaw / (pi / 2))
        print(f"Yaw: {yaw}")
        print(f"rounded to: {direction}")
        return direction, yaw
