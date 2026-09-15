from dataclasses import dataclass
from typing import TypedDict


class TMovement(TypedDict):
    frames: int
    left: int
    right: int


@dataclass(frozen=True)
class Movement:
    frames_per_move: int = 60
    frames_per_rotation: int = 25
    forward_speed: int = 5
    rotation_speed: int = 3

    def forward(self) -> TMovement:
        return {
            "frames": self.frames_per_move,
            "left": self.forward_speed,
            "right": self.forward_speed,
        }

    def turn_left(self) -> TMovement:
        return {
            "frames": self.frames_per_rotation,
            "left": -self.rotation_speed,
            "right": self.rotation_speed,
        }

    def turn_right(self) -> TMovement:
        return {
            "frames": self.frames_per_rotation,
            "left": self.rotation_speed,
            "right": -self.rotation_speed,
        }
