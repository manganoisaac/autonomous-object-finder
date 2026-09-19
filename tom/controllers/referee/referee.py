"""Referee supervisor: ends the run at a sim-time limit and records the outcome.

Uses simulator ground truth, which is fine here: it only scores the run and
never talks to the robot controller.
"""
import json
import os
import time

from controller import Supervisor

LIMIT = float(os.environ.get("BENCH_SIM_SECONDS", "240"))
OUT = os.environ.get("BENCH_OUT", "")

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
start = time.time()

epuck = None
children = supervisor.getRoot().getField("children")
for i in range(children.getCount()):
    node = children.getMFNode(i)
    name = node.getField("name")
    if name is not None and name.getSFString() == "e-puck":
        epuck = node
        break

contact_steps = 0
while supervisor.step(timestep) != -1:
    if epuck is not None and epuck.getContactPoints(True):
        contact_steps += 1
    if supervisor.getTime() >= LIMIT:
        wall = time.time() - start
        result = {
            "sim_seconds": round(supervisor.getTime(), 3),
            "wall_seconds": round(wall, 3),
            "rtf": round(supervisor.getTime() / wall, 3),
            "contact_steps": contact_steps,
            "final_pose": list(epuck.getPosition()) if epuck else None,
        }
        print("REFEREE_RESULT " + json.dumps(result), flush=True)
        if OUT:
            with open(OUT, "w") as fh:
                json.dump(result, fh)
        supervisor.simulationQuit(0)
        break
