from dataclasses import dataclass
from math import pi
from typing import Any

from controller import Robot
from gps import GPS
from movement import Movement, TMovement


@dataclass(frozen=True)
class RobotController:
    robot: Robot
    timestep: Any
    gps: GPS

    def move(self, set_speed) -> tuple[int, int]:

        # get initial coordinate
        self.robot.step(self.timestep)
        starting_cell = self.gps.coordinate()
        print(f"At coordinate: {self.gps.coordinate()}")

        # set robot speed
        set_speed(3, 3)
        print(f"moving forward")

        # avoid glitchy border behaviour
        for _ in range(20):
            self.robot.step(self.timestep)

        # just testing (candidate replacement)
        starting_cell = self.gps.coordinate()

        # move until in new cell
        while self.gps.coordinate() == starting_cell:
            self.robot.step(self.timestep)

        # avoid glitchy border behaviour
        for _ in range(20):
            self.robot.step(self.timestep)

        # reset speed to none
        set_speed(0, 0)
        return self.gps.coordinate()

    def turn_left(self, set_speed):
        print("turning left")
        # get starting direction
        self.robot.step(self.timestep)
        set_speed(-3, 3)

        # round to nearest pi / 2
        starting_direction, _ = self.gps.direction()
        print(f"starting direction: {starting_direction}")

        # between (pi, 2 * pi) [-pi, -(pi / 2), 0, (pi / 2)]
        starting_yaw = (starting_direction * (pi / 2) + pi) % (2 * pi)

        # between (pi, 2 * pi), could have overflowed down
        target_yaw = (starting_yaw + (pi / 2)) % (2 * pi)

        while True:
            new_yaw = self.gps.direction()[1] + pi
            if abs(target_yaw - new_yaw) < 0.1:
                break
            self.robot.step(self.timestep)

        set_speed(0, 0)

    def turn_right(self, set_speed):
        print("turning right")
        # get starting direction
        self.robot.step(self.timestep)
        set_speed(3, -3)

        # round to nearest pi / 2
        starting_direction, _ = self.gps.direction()
        print(f"starting direction: {starting_direction}")

        # between (pi, 2 * pi) [-pi, -(pi / 2), 0, (pi / 2)]
        starting_yaw = (starting_direction * (pi / 2) + pi) % (2 * pi)

        # between (pi, 2 * pi), could have overflowed down
        target_yaw = (starting_yaw - (pi / 2)) % (2 * pi)

        while True:
            new_yaw = self.gps.direction()[1] + pi
            if abs(target_yaw - new_yaw) < 0.1:
                break
            self.robot.step(self.timestep)

        set_speed(0, 0)
