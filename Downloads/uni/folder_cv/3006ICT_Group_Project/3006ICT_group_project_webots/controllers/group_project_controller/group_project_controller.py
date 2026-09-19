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
import torch
import torchvision
from controller import Robot
from torchvision import transforms

from detector.model import Detector
from gps import GPS
from movement import Movement, TMovement
from path import COMMAND, Path
from project_utils import CONFIG, ROOT, grid_to_world, world_to_grid
from robot import RobotController
from stations import Station, Stations

# ----------
# CNN setup
# ----------
model = torch.load("./detector/detector_model.pth", weights_only=False)
model.eval()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"model: {model}")
model = model.to(device)
transform = transforms.Compose(
    [
        transforms.ToPILImage(),
        transforms.Resize(256),
        transforms.CenterCrop(244),
        transforms.ToTensor(),
        transforms.Normalize([0.5, 0.5, 0.5], [0.2, 0.2, 0.2]),
    ]
)
CLASSES = [
    "backpack",
    "camera",
    "wall_clock",
    "coffee_mug",
    "fire_extinguisher",
    "headphones",
    "running_shoe",
    "soda_can",
    # # correct names
    # "soda_can",
    # "coffee_mug",
    # "backpack",
    # "fire_extinguisher",
    # "camera",
    # "running_shoe",
    # "headphones",
    # "wall_clock",
]
detector = Detector(model, transform, CLASSES, device)


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
# TODO: check -1 or 1?
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
        img = camera_bgr()
        predicted_target = detector.detect(img)
        print(f"predicted_target: {predicted_target}")
        if predicted_target == target:
            print(f"Found target: {target}")
            break
        else:
            print(f"Not our target. Search continues...")


if __name__ == "__main__":
    main()
