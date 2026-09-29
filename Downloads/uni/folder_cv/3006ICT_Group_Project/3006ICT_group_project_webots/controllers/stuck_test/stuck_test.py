"""
Fault-injection test for stuck detection (test tool only - not part of the mission).

Uses the REAL RobotController.move() from the mission controller, but can feed it
"blind" proximity readings (a constant noise-level value), so the proximity safety
layer never fires. The robot then drives into an obstacle, and the only thing that
can stop it is the stuck detector (progress monitoring).

Usage
-----
1. Open worlds/safety_test_A.wbt and set the e-puck `controller` field to "stuck_test".
2. Use the same poses as the safety tests:
     translation -1.5 -0.6 0
     rotation 0 0 1 3.14159   (head-on)   or   0 0 1 2.35619   (45 degrees)
3. Run and read the console.
"""

import math
import sys
from pathlib import Path

import numpy as np
from controller import Robot

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "group_project_controller"))
from gps import GPS  # noqa: E402
from robot import RobotController  # noqa: E402

# --- settings -----------------------------------------------------------
BLIND_SENSORS = True   # True: proximity layer disabled (fault injection)
BLIND_VALUE = 67.0     # mean full-mission noise level measured on all sensors
MAX_MOVES = 30         # safety cap on the number of one-cell moves
MAX_SPEED = 10
# ------------------------------------------------------------------------

robot = Robot()
ts = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

ps = [robot.getDevice(f"ps{i}") for i in range(8)]
for sensor in ps:
    sensor.enable(ts)

raw_gps = robot.getDevice("gps")
imu = robot.getDevice("imu")
raw_gps.enable(ts)
imu.enable(ts)


def set_speed(left, right):
    left_motor.setVelocity(float(np.clip(left, -MAX_SPEED, MAX_SPEED)))
    right_motor.setVelocity(float(np.clip(right, -MAX_SPEED, MAX_SPEED)))


def real_proximity():
    return [s.getValue() for s in ps]


def get_proximity():
    return [BLIND_VALUE] * 8 if BLIND_SENSORS else real_proximity()


gps = GPS(gps=raw_gps, imu=imu, map=[[]])
controller = RobotController(robot=robot, timestep=ts, gps=gps)

robot.step(ts)
x0, y0, _ = raw_gps.getValues()
print(f"Stuck test started (BLIND_SENSORS={BLIND_SENSORS})")

for i in range(MAX_MOVES):
    cell = gps.coordinate()
    events = controller.move(set_speed, get_proximity)
    x, y, _ = raw_gps.getValues()
    travelled = math.hypot(x - x0, y - y0)
    names = [e.name for e in events]
    print(f"move {i + 1:2d}: {cell} -> {gps.coordinate()}  travelled {travelled:.4f} m  events {names}")

    if events:
        peak = max(real_proximity())
        print(f"TEST RESULT: stopped by {names} after {travelled:.4f} m")
        print(f"(real proximity peak at stop: {peak:.1f} - what the blinded sensors actually saw)")
        break
else:
    print(f"TEST RESULT: no event within {MAX_MOVES} moves")

set_speed(0, 0)
while robot.step(ts) != -1:
    pass
