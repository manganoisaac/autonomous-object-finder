from dataclasses import dataclass
from math import hypot, pi
from typing import Any

import numpy as np
from controller import Robot

from gps import GPS
from movement import Movement, TMovement
from safety.safety import MotionEvent, ProximityDirection, check_proximity

# ---------------------------------------------------------------
# Forward motion and stuck detection
# ---------------------------------------------------------------
FORWARD_SPEED = 6.27  # rad/s
WHEEL_RADIUS = 0.02  # m; gives 4.0 mm/step at 32 ms, matching measured free motion

# Stuck rule: the move is stuck if NONE of the last STUCK_WINDOW steps reached
# SLOW_STEP_RATIO of the expected free-motion step (4.0 mm -> 3.0 mm).
#
# Calibrated from logs:
#   contact, sliding along a wall   1.8-2.3 mm every step   (safety_diagnostic)
#   contact, blocked head-on        ~0 mm every step        (safety_diagnostic)
#   normal first move after a turn  irregular 1.9-4.0 mm; at most 4 slow steps
#                                   in a row before a full-speed step (Start C)
# A window of 8 gives 2x margin over normal motion and detects contact
# ~8 steps (~0.26 s) after it happens.
STUCK_WINDOW = 8  # steps
SLOW_STEP_RATIO = 0.75
MAX_STEPS_PER_MOVE = 75  # ~3x the ~25 steps one 0.1 m cell normally takes


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
    ) -> set:
        if global_dir_label == "":
            return set()

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
        print("REVERSE RECOVERY complete.")

        return True

    def step_back(self, set_speed) -> None:
        set_speed(-6.27, -6.27)
        for _ in range(30):
            self.robot.step(self.timestep)

    def _position(self) -> tuple[float, float]:
        x, y, _ = self.gps.gps.getValues()
        return x, y

    def _stuck_reason(self, history: list[tuple[float, float]]) -> str | None:
        """
        Return why the current move counts as stuck, or None if it is progressing.

        history holds the (x, y) position after every step of the current move.
        """
        if len(history) > MAX_STEPS_PER_MOVE:
            return f"no new cell after {MAX_STEPS_PER_MOVE} steps"

        if len(history) <= STUCK_WINDOW:
            return None

        recent = history[-1 - STUCK_WINDOW :]
        steps = [hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(recent, recent[1:])]
        expected_step = FORWARD_SPEED * WHEEL_RADIUS * (self.timestep / 1000)
        slow_step = SLOW_STEP_RATIO * expected_step

        if max(steps) < slow_step:
            return (
                f"no step above {slow_step * 1000:.1f} mm in the last {STUCK_WINDOW} "
                f"steps (max {max(steps) * 1000:.1f} mm)"
            )
        return None

    def _forward_step(self, set_speed, get_proximity, history) -> set | None:
        """
        Advance one simulation step and run both safety layers.

        Returns the event set if the move must stop, otherwise None.
        """
        self.robot.step(self.timestep)

        # Layer 1: proximity sensors (sees obstacles before contact).
        dangers = check_proximity(get_proximity())
        if dangers:
            set_speed(0, 0)

            # a pure FRONT danger triggers a short reverse recovery
            if dangers == {ProximityDirection.FRONT}:
                self.recover_from_front(set_speed, get_proximity)
            return dangers

        # Layer 2: progress monitoring (catches what the sensors miss).
        history.append(self._position())
        reason = self._stuck_reason(history)
        if reason:
            set_speed(0, 0)
            print(f"STUCK: {reason}")
            self.recover_from_front(set_speed, get_proximity)
            return {MotionEvent.STUCK}

        return None

    def move(self, set_speed, get_proximity) -> set:
        """
        Drive forward into the next grid cell.

        Returns an empty set on success, otherwise the events that stopped
        the move (ProximityDirection values and/or MotionEvent.STUCK).
        """
        history: list[tuple[float, float]] = []

        set_speed(FORWARD_SPEED, FORWARD_SPEED)

        # Keep the original short initial movement.
        for _ in range(3):
            events = self._forward_step(set_speed, get_proximity, history)
            if events:
                return events

        # Candidate replacement for the initial coordinate.
        starting_cell = self.gps.coordinate()

        # move until entering a new grid cell
        while self.gps.coordinate() == starting_cell:
            events = self._forward_step(set_speed, get_proximity, history)
            if events:
                return events

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

    def right_for_radians(self, set_speed, radians: float = 0.0873):
        _, starting_yaw = self.gps.direction()
        target_yaw = (((starting_yaw + pi) - radians) % (2 * pi)) - pi

        while abs(self.gps.direction()[1] - target_yaw) > 0.05:
            set_speed(0.5, -0.5)
            self.robot.step(self.timestep)
        set_speed(0, 0)

    def left_for_radians(self, set_speed, radians: float = 0.0873):
        _, starting_yaw = self.gps.direction()
        target_yaw = (((starting_yaw + pi) + radians) % (2 * pi)) - pi

        while abs(self.gps.direction()[1] - target_yaw) > 0.05:
            set_speed(-0.5, 0.5)
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

        # New version
        # ===========
        _, cur_yaw = self.gps.direction()
        diff = ((global_yaw - cur_yaw + pi) % (2 * pi)) - pi
        print(f"DEBUG[diff]: {diff}")

        while abs(diff) > 0.05:

            if diff > 0:
                print("diff greater than 0")
                set_speed(-2, 2)
            else:
                print("diff lesser than 0")
                set_speed(2, -2)

            self.robot.step(self.timestep)
            _, cur_yaw = self.gps.direction()
            diff = ((global_yaw - cur_yaw + pi) % (2 * pi)) - pi
            print(f"DEBUG[diff]: {diff}")

        set_speed(0, 0)
        return
