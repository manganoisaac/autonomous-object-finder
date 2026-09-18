from dataclasses import dataclass
from math import pi
from typing import Any

import numpy as np
from controller import Robot
from gps import GPS
from movement import Movement, TMovement


@dataclass(frozen=True)
class RobotController:
    robot: Robot
    timestep: Any
    gps: GPS

    def move_from_global_direction(self, global_dir_label: str, set_speed) -> None:
        if global_dir_label == "":
            return

        # TODO: make a direction enum (UP, DOWN, LEFT, RIGHT)
        class Direction:
            RIGHT = [[1, 0], [0, 1]]
            UP = [[0, 1], [-1, 0]]
            DOWN = [[0, -1], [1, 0]]
            LEFT = [[-1, 0], [0, -1]]

        class Turn:
            STRAIGHT = Direction.RIGHT
            LEFT = Direction.UP
            RIGHT = Direction.DOWN
            BACK = Direction.LEFT

        direction_to_matrix = {
            0: Direction.RIGHT,
            1: Direction.UP,
            2: Direction.LEFT,
            -2: Direction.LEFT,
            -1: Direction.DOWN,
        }

        label_to_matrix = {
            "RIGHT": Direction.RIGHT,
            "UP": Direction.UP,
            "LEFT": Direction.LEFT,
            "DOWN": Direction.DOWN,
        }

        # get initial direction
        starting_direction, _ = self.gps.direction()
        print(f"starting direction: {starting_direction}")
        robot_direction = direction_to_matrix[starting_direction]
        global_direction = label_to_matrix[global_dir_label]
        print(f"robot matrix: {robot_direction}")
        print(f"global matrix: {global_direction}")

        # inverse of robot direction matmul global direction
        # ...
        RD = np.array(robot_direction)
        GD = np.array(global_direction)
        rotation_needed = np.linalg.inv(RD) @ GD

        print(f"Rotation Needed: {rotation_needed}")

        match rotation_needed.astype(int).tolist():
            case Turn.STRAIGHT:
                self.move(set_speed)
                return
            case Turn.RIGHT:
                print(f"Turning right")
                self.turn_right(set_speed)
                self.move(set_speed)
                return
            case Turn.LEFT:
                print(f"Turning left")
                self.turn_left(set_speed)
                self.move(set_speed)
                return
            case Turn.BACK:
                print(f"Turning backwards")
                self.turn_left(set_speed)
                self.turn_left(set_speed)
                self.move(set_speed)
                return

    def move(self, set_speed) -> tuple[int, int]:

        # get initial coordinate
        starting_cell = self.gps.coordinate()
        print(f"At coordinate: {self.gps.coordinate()}")

        # set robot speed
        set_speed(6.27, 6.27)
        print(f"moving forward")
        for _ in range(3):
            self.robot.step(self.timestep)

        # just testing (candidate replacement)
        starting_cell = self.gps.coordinate()

        # move until in new cell
        while self.gps.coordinate() == starting_cell:
            self.robot.step(self.timestep)

        # reset speed to none
        set_speed(0, 0)
        return self.gps.coordinate()

    def turn_left(self, set_speed):
        print("turning left")
        # get starting direction
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
