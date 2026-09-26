from dataclasses import dataclass
from math import pi
from typing import Any

import numpy as np
from controller import Robot

from gps import GPS
from movement import Movement, TMovement
from safety.safety import check_proximity, ProximityDirection


@dataclass(frozen=True)
class RobotController:
    robot: Robot
    timestep: Any
    gps: GPS

    def move_from_global_direction(
        self,
        global_dir_label: str,
        set_speed,
        get_proximity,
    ) -> set[ProximityDirection]:
        if global_dir_label == "":
            return set()

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
        robot_direction = direction_to_matrix[starting_direction]
        global_direction = label_to_matrix[global_dir_label]

        # Determine rotation required to face the global direction
        RD = np.array(robot_direction)
        GD = np.array(global_direction)
        rotation_needed = np.linalg.inv(RD) @ GD

        match rotation_needed.astype(int).tolist():
            case Turn.STRAIGHT:
                return self.move(set_speed, get_proximity)
            case Turn.RIGHT:
                self.turn_right(set_speed)
                return self.move(set_speed, get_proximity)
            case Turn.LEFT:
                self.turn_left(set_speed)
                return self.move(set_speed, get_proximity)
            case Turn.BACK:
                self.turn_left(set_speed)
                self.turn_left(set_speed)
                return self.move(set_speed, get_proximity)

        # defensive fallback
        return set()

    def recover_from_front(self, set_speed, get_proximity) -> bool:
        """
        Perform a short reverse movement after a FRONT danger event.

        Returns True if recovery completed successfully.
        Returns False if reversing becomes unsafe.
        """

        # Check whether reversing is safe before starting.
        current_dangers = check_proximity(get_proximity())

        if ProximityDirection.BACK in current_dangers:
            set_speed(0, 0)
            print("RECOVERY BLOCKED: BACK is also dangerous.")
            return False

        # Reverse more slowly than normal forward movement.
        set_speed(-3.0, -3.0)

        # Short conservative reverse recovery.
        for _ in range(4):
            self.robot.step(self.timestep)

            dangers = check_proximity(get_proximity())

            # Stop immediately if something becomes dangerous behind us.
            if ProximityDirection.BACK in dangers:
                set_speed(0, 0)
                print("RECOVERY STOP: BACK danger detected.")
                return False

        set_speed(0, 0)
        print("FRONT RECOVERY complete.")

        return True
        
    def step_back(self, set_speed) -> None:
        set_speed(-6.27, -6.27)
        for _ in range(30):
            self.robot.step(self.timestep)

    def move(self, set_speed, get_proximity) -> set[ProximityDirection]:

        # get initial coordinate
        starting_cell = self.gps.coordinate()

        # set robot speed
        set_speed(6.27, 6.27)

        # Keep the original short initial movement, while checking
        # proximity after every simulation step
        for _ in range(3):
            self.robot.step(self.timestep)

            dangers = check_proximity(get_proximity())
            if dangers:
                set_speed(0, 0)

                # a pure FRONT danger triggers a short reverse recovery
                if dangers == {ProximityDirection.FRONT}:
                    self.recover_from_front(
                        set_speed,
                        get_proximity,
                    )
                    
                return dangers

        # Candidate replacement for the initial coordinate.
        starting_cell = self.gps.coordinate()

        # move until entering a new grid cell 
        while self.gps.coordinate() == starting_cell:
            self.robot.step(self.timestep)

            # Read the latest proximity values on every simulation step.
            dangers = check_proximity(get_proximity())

            # Stop the current movement and report the detected
            # danger directions. FRONT danger also triggers a short recovery.
            if dangers:
                set_speed(0, 0)

                if dangers == {ProximityDirection.FRONT}:
                    self.recover_from_front(
                        set_speed, 
                        get_proximity,
                    )
                return dangers

        # stop after completing the movement 
        set_speed(0, 0)

        # Empty set means the move completed without a safety event.
        return set()

    def turn_left(self, set_speed):
        set_speed(-3, 3)

        # round to nearest pi / 2
        starting_direction, _ = self.gps.direction()

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
        set_speed(3, -3)

        # round to nearest pi / 2
        starting_direction, _ = self.gps.direction()

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

    def go_to_yaw(self, global_yaw, set_speed):
        set_speed(-3, 3)

        _, cur_yaw = self.gps.direction()
        while abs(cur_yaw - global_yaw) > 0.05:
            self.robot.step(self.timestep)
            _, cur_yaw = self.gps.direction()
        set_speed(0, 0)
