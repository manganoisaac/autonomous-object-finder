from dataclasses import dataclass
from math import floor, pi
from typing import Any

from project_utils import get_cell


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
        return get_cell(x, y)

    def direction(self) -> tuple[int, float]:
        yaw = self.imu.getRollPitchYaw()[2]
        direction = round(yaw / (pi / 2))
        return direction, yaw
