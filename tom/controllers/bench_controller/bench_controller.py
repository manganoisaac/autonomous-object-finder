"""Stand-in load for throughput benchmarks: camera + ORB matching + reactive driving.

Not a mission solver. It exists so the benchmark pays a realistic per-step
perception cost until the real controller is ready to be swapped in.
"""
import glob
import os
import time

import cv2
import numpy as np
from controller import Robot

from project_utils import ROOT

CV_EVERY = int(os.environ.get("BENCH_CV_EVERY", "4"))
MAX_SPEED = 6.28

robot = Robot()
timestep = int(robot.getBasicTimeStep())
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
camera = robot.getDevice("camera")
ps = [robot.getDevice(f"ps{i}") for i in range(8)]
for device in [camera, robot.getDevice("gps"), robot.getDevice("imu")] + ps:
    device.enable(timestep)
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

orb = cv2.ORB_create(nfeatures=300)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
references = []
for path in sorted(glob.glob(str(ROOT / "textures" / "target_*.png"))):
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    _, descriptors = orb.detectAndCompute(cv2.resize(image, (160, 160)), None)
    if descriptors is not None:
        references.append(descriptors)

steps = 0
max_std = 0.0
start = time.time()
while robot.step(timestep) != -1:
    steps += 1
    raw = camera.getImage()
    if raw:
        bgra = np.frombuffer(raw, np.uint8).reshape(camera.getHeight(), camera.getWidth(), 4)
        max_std = max(max_std, float(bgra[:, :, :3].std()))
        if steps % CV_EVERY == 0:
            _, descriptors = orb.detectAndCompute(cv2.cvtColor(bgra, cv2.COLOR_BGRA2GRAY), None)
            if descriptors is not None and len(descriptors) > 4:
                for reference in references:
                    matcher.match(descriptors, reference)
    values = [sensor.getValue() for sensor in ps]
    left = right = 0.6 * MAX_SPEED
    if values[0] > 120 or values[7] > 120:
        left, right = -0.3 * MAX_SPEED, 0.5 * MAX_SPEED
    elif values[5] > 120 or values[6] > 120:
        left, right = 0.5 * MAX_SPEED, -0.3 * MAX_SPEED
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)
    if steps % 500 == 0:
        elapsed = time.time() - start
        print(f"BENCH steps={steps} simt={robot.getTime():.1f} wall={elapsed:.1f} "
              f"rtf={robot.getTime() / elapsed:.2f} camstd={max_std:.1f}", flush=True)
