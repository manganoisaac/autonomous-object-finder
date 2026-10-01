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

# ---------------------------------------------------------------
# Webots setup
# ---------------------------------------------------------------
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


# ---------------------------------------------------------------
# Provided low-level helpers
# ---------------------------------------------------------------
def set_speed(left, right):
    """
    Set the left and right wheel speeds.
    """
    left = np.clip(left, -MAX_SPEED, MAX_SPEED)
    right = np.clip(right, -MAX_SPEED, MAX_SPEED)

    left_motor.setVelocity(float(left))
    right_motor.setVelocity(float(right))


def get_pose():
    """
    Return provided ground-truth-like pose (x, y, yaw).
    """
    x, y, _ = raw_gps.getValues()
    yaw = imu.getRollPitchYaw()[2]

    return x, y, yaw


def camera_bgr():
    h = camera.getHeight()
    w = camera.getWidth()

    image = np.frombuffer(camera.getImage(), np.uint8).reshape(h, w, 4)

    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


PS_MAX = [0.0] * 8
PS_MAX_CELL = [None] * 8


def proximity_values():
    values = [sensor.getValue() for sensor in ps]
    for i, v in enumerate(values):
        if v > PS_MAX[i]:
            PS_MAX[i] = v
            PS_MAX_CELL[i] = gps.coordinate()
    return values


def get_best_confidence_prediction(controller, detector) -> str:
    images = []
    controller.left_for_radians(set_speed)
    image = camera_bgr()
    images.append(image)
    controller.left_for_radians(set_speed)
    image = camera_bgr()
    images.append(image)
    controller.right_for_radians(set_speed)
    controller.right_for_radians(set_speed)
    controller.right_for_radians(set_speed)
    image = camera_bgr()
    images.append(image)
    controller.right_for_radians(set_speed)
    image = camera_bgr()
    images.append(image)
    class_label = detector.detect_best(images)
    print()
    print(f"DETECTION")
    print(f"-" * 20)
    print(class_label)
    print()
    return class_label.label


# ------------------------------------------------------------------
# Group implementation
# ------------------------------------------------------------------
# TO DO


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    print("Group-project controller started.")
    print("Mission:", MISSION)
    print("target:", target)
    print("Stations:", [s["id"] for s in CONFIG["stations"]])
    print("Camera:", camera.getWidth(), "x", camera.getHeight())
    print("before path")
    print(f"stations: {CONFIG['stations']}")
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
            direction_labels = path.path_a_star(starting_position, goal)
            for direction_label in direction_labels:
                cell, label = direction_label
                if cell != gps.coordinate():
                    break
                dangers = controller.move_from_global_direction(
                    label,
                    set_speed,
                    proximity_values,
                )

                if dangers:
                    print(
                        "SAFETY EVENT:",
                        [direction.name for direction in dangers],
                    )
                    print(
                        "Replanning from:",
                        gps.coordinate(),
                    )

                    # RobotController has already stopped/recovered.
                    # Exit the current path so it can be recalculated
                    # from the robot's updated position.
                    break

        controller.go_to_yaw(yaw, set_speed)
        predicted_target = get_best_confidence_prediction(controller, detector)
        print(f"predicted_target: {predicted_target}")
        if predicted_target == target:
            print(f"Found target: {target}")
            break
        else:
            print(f"Not our target. Search continues...")

    # Evaluation log only (not used for any decision):
    # distance from the final pose to the observation position of the
    # station the robot stopped at. Success criterion: <= 0.20 m.
    x, y, _ = raw_gps.getValues()
    stop_station = next(
        s for s in CONFIG["stations"] if world_to_grid(*s["observe"]) == goal
    )
    ox, oy = stop_station["observe"]
    print(f"MISSION TIME: {robot.getTime():.1f} s (simulation)")
    print(
        f"FINAL DISTANCE to {stop_station['id']} observation position: "
        f"{math.hypot(x - ox, y - oy):.3f} m"
    )

    print("PROXIMITY PEAKS (value @ cell):")
    for i in range(8):
        print(f"  ps{i}: {PS_MAX[i]:.1f} @ {PS_MAX_CELL[i]}")

    # Motors retain their last command when main() returns.
    set_speed(0, 0)
    while robot.step(timestep) != -1:
        pass


if __name__ == "__main__":
    main()
