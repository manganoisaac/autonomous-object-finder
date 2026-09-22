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

from detector import Detector
from gps import GPS
from movement import Movement, TMovement
from path import COMMAND, Path
from project_utils import CONFIG, ROOT, grid_to_world, world_to_grid
from robot import RobotController
from safety.safety import is_dangerously_close
from stations import Station, Stations

detector = Detector()


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

# For CNN
# ---------

target = MISSION["target"]

gps = GPS(gps=raw_gps, imu=imu, map=[[]])
controller = RobotController(robot=robot, timestep=timestep, gps=gps)
path = Path(checkpoints=[], map=GRID[::1])


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
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


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
    print(f"stations: {CONFIG["stations"]}")
    stations = Stations(
        [(st["observe"], st["observe_yaw"], st["id"]) for st in CONFIG["stations"]]
    )
    robot.step(timestep)
    while len(stations) > 0:
        retries = 0
        cur_station = stations.pop(gps.coordinate())
        goal, yaw = cur_station
        while gps.coordinate() != goal and retries < 100:
            retries += 1
            starting_position = gps.coordinate()
            directon_labels = path.path(starting_position, goal)
            for direction_label in directon_labels:
                cell, label = direction_label
                if cell != gps.coordinate():
                    break
                controller.move_from_global_direction(label, set_speed)

        controller.go_to_yaw(yaw, set_speed)
        print(f"proximity_values: {proximity_values()}")
        if not is_dangerously_close(proximity_values()):
            print("stepping back...")
            controller.step_back(set_speed)
        seen = detector.detect(camera_bgr())
        print(f"seen: {seen.label} ({seen.confidence:.2f})")
        if seen.label == target:
            print(f"Found target: {target}")
            break
        else:
            print(f"Not our target. Search continues...")

    # The motors keep their last command once main() returns, and step_back() leaves them
    # reversing, so the robot would drive away from the station it just identified.
    set_speed(0, 0)
    while robot.step(timestep) != -1:
        pass


if __name__ == "__main__":
    main()
