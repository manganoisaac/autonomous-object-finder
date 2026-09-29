"""
Safety diagnostic controller (test tool only - not part of the mission controller).

Drives the e-puck straight ahead at mission speed and logs all eight proximity
sensors plus distance travelled on every simulation step, until the robot makes
contact with an obstacle (detected as "no forward progress").

Clearance for every logged step is then:
    clearance = travelled_at_contact - travelled_at_step

Usage
-----
1. Use a COPY of a training world (e.g. worlds/safety_test.wbt).
2. Set the e-puck's `controller` field to "safety_diagnostic".
3. Place the e-puck ~0.3 m from an obstacle, facing it head-on or at ~45 degrees.
4. Run. Results are written to safety_log_<label>.csv in this folder.
"""

import csv
import math
import sys
from pathlib import Path

from controller import Robot

# Reuse the mission controller's safety module (no copy, so we test the real code).
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "group_project_controller"))
from safety.safety import check_proximity  # noqa: E402

# --- settings -----------------------------------------------------------
LABEL = "diag45_check"        # change per run, e.g. "head_on", "diag45_left"
SPEED = 6.27            # rad/s - same forward speed as RobotController.move()
MAX_STEPS = 3000
CONTACT_WINDOW = 10     # steps with (almost) no progress = contact
CONTACT_EPS = 0.0003    # metres; progress below this over the window = stuck

# True  -> stop at the first safety event from safety.check_proximity()
#          (verification run: compare travelled distance with the contact
#           distance from the earlier logging run at the same start pose)
# False -> drive until contact (calibration / logging run)
SAFETY_CHECK = True
# ------------------------------------------------------------------------

robot = Robot()
ts = int(robot.getBasicTimeStep())

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
for motor in (left, right):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

ps = [robot.getDevice(f"ps{i}") for i in range(8)]
for sensor in ps:
    sensor.enable(ts)

gps = robot.getDevice("gps")
gps.enable(ts)

robot.step(ts)
x0, y0, _ = gps.getValues()

rows = []
history = []

left.setVelocity(SPEED)
right.setVelocity(SPEED)

for step in range(MAX_STEPS):
    if robot.step(ts) == -1:
        break

    x, y, _ = gps.getValues()
    travelled = math.hypot(x - x0, y - y0)
    values = [s.getValue() for s in ps]
    rows.append([step, travelled] + values)

    if SAFETY_CHECK:
        dangers = check_proximity(values)
        if dangers:
            names = sorted(d.name for d in dangers)
            print(f"SAFETY EVENT {names} at step {step}, travelled {travelled:.4f} m")
            break

    history.append(travelled)
    if len(history) > CONTACT_WINDOW and step > 20:
        if history[-1] - history[-1 - CONTACT_WINDOW] < CONTACT_EPS:
            print(f"CONTACT detected at step {step}, travelled {travelled:.4f} m")
            break

left.setVelocity(0.0)
right.setVelocity(0.0)

# Clearance relative to the contact point (last recorded position).
contact = rows[-1][1] if rows else 0.0
out = Path(__file__).with_name(f"safety_log_{LABEL}.csv")
with out.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["step", "travelled_m", "clearance_m"] + [f"ps{i}" for i in range(8)])
    for row in rows:
        step, travelled, *values = row
        writer.writerow(
            [step, f"{travelled:.4f}", f"{contact - travelled:.4f}"]
            + [f"{v:.1f}" for v in values]
        )

print(f"Saved {len(rows)} rows to {out}")

while robot.step(ts) != -1:
    pass