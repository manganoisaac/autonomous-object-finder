"""
3006ICT Group Project - Controller

Mission:
    Search the observation stations, identify the requested visual target,
    navigate safely, and stop at the correct target.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
from controller import Robot
from gps import GPS
from movement import Movement, TMovement
from path import COMMAND, Path
from project_utils import CONFIG, ROOT, grid_to_world, world_to_grid
from robot import RobotController

# ------------------------------------------------------------------
# Webots setup
# ------------------------------------------------------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())
movement = Movement()

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
camera = robot.getDevice("camera")
ps = [robot.getDevice(f"ps{i}") for i in range(8)]
raw_gps = robot.getDevice("gps")
imu = robot.getDevice("imu")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

camera.enable(timestep)
raw_gps.enable(timestep)
imu.enable(timestep)
for sensor in ps:
    sensor.enable(timestep)

MAX_SPEED = 10
GRID = np.load(ROOT / "maps" / "occupancy_grid.npy")
MISSION = json.loads((ROOT / "config" / "assessment_mission.json").read_text())
target = MISSION["target"]

gps = GPS(gps=raw_gps, imu=imu, map=[[]])
controller = RobotController(robot=robot, timestep=timestep, gps=gps)
path = Path(checkpoints=[], map=GRID[::-1])


# ------------------------------------------------------------------
# Provided low-level helpers
# ------------------------------------------------------------------
def set_speed(left, right):
    left = np.clip(left, -MAX_SPEED, MAX_SPEED)
    right = np.clip(right, -MAX_SPEED, MAX_SPEED)
    left_motor.setVelocity(float(left))
    right_motor.setVelocity(float(right))


def get_pose():
    """Return provided ground-truth-like pose (x, y, yaw)."""
    x, y, _ = raw_gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    return x, y, yaw


def camera_bgr():
    h, w = camera.getHeight(), camera.getWidth()
    image = np.frombuffer(camera.getImage(), np.uint8).reshape(h, w, 4)
    return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)


def proximity_values():
    return [sensor.getValue() for sensor in ps]


# ------------------------------------------------------------------
# Group implementation
# ------------------------------------------------------------------
# TO DO


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    print("Group-project controller started.")
    print("Mission:", MISSION)
    print("target:", target)
    print("Stations:", [s["id"] for s in CONFIG["stations"]])
    print("Camera:", camera.getWidth(), "x", camera.getHeight())
    print("before path")
    robot.step(timestep)
    retries = 0
    while gps.coordinate() != (16, 24) and retries < 100:
        retries += 1
        starting_position = gps.coordinate()
        directon_labels = path.path(starting_position)
        print(f"grid: {GRID}")
        print(f"grid shape: {GRID.shape}")
        print("after path")
        print("Path Cells:")
        print(directon_labels)
        for direction_label in directon_labels:
            cell, label = direction_label
            print(f"should be at cell {cell}")
            if cell != gps.coordinate():
                print(f"cells don't match:")
                print(f"should be: {cell}")
                print(f"actual location: {gps.coordinate()}")
                break
            controller.move_from_global_direction(label, set_speed)

    print(f"WE ARE AT CELL {gps.coordinate()}")

    # commands = [
    #     COMMAND.MOVE,
    #     COMMAND.TURN_LEFT,
    #     COMMAND.MOVE,
    #     COMMAND.TURN_RIGHT,
    #     COMMAND.MOVE,
    #     COMMAND.TURN_LEFT,
    #     COMMAND.MOVE,
    #     COMMAND.TURN_LEFT,
    #     COMMAND.MOVE,
    #     COMMAND.TURN_LEFT,
    #     COMMAND.MOVE,
    #     COMMAND.TURN_RIGHT,
    #     COMMAND.MOVE,
    # ]
    #
    # for command in commands:
    #     match command:
    #         case COMMAND.TURN_LEFT:
    #             controller.turn_left(set_speed=set_speed)
    #         case COMMAND.TURN_RIGHT:
    #             controller.turn_right(set_speed=set_speed)
    #         case COMMAND.MOVE:
    #             controller.move(set_speed=set_speed)

    # while True:
    #     controller.move(set_speed=set_speed)
    #     controller.turn_left(set_speed=set_speed)
    #     controller.move(set_speed=set_speed)
    #     controller.turn_right(set_speed=set_speed)


if __name__ == "__main__":
    main()
