# Autonomous Object Finder – Integrated System Summary

## Overview

This project implements an autonomous e-puck robot in Webots that searches for a specified target object across eight observation stations.

The integrated system combines:

- mission parsing;
- GPS/IMU-based localisation;
- station selection;
- grid-based path planning;
- robot movement control;
- multi-view visual perception;
- directional proximity-based safety monitoring;
- front-obstacle recovery; and
- replanning after safety events.

The mission target is defined in the mission configuration file. The robot autonomously navigates between observation stations, observes each station using its onboard RGB camera, classifies the observed object, and continues searching until the predicted object matches the mission target.

The system was tested using all three provided starting positions and all eight target classes.

---

## System Architecture

The main components of the integrated controller are:

### Mission

The controller reads the target object from the mission configuration file.

Example:

```json
{
  "target": "soda_can"
}
```

Supported target classes include:

- `soda_can`
- `coffee_mug`
- `backpack`
- `fire_extinguisher`
- `camera`
- `running_shoe`
- `headphones`
- `wall_clock`

---

## Localisation

The robot uses:

- GPS for position; and
- an Inertial Unit for orientation.

Robot position is converted into grid-cell coordinates for navigation and station selection.

---

## Observation Station Selection

The robot selects the next observation station using a greedy nearest-station strategy.

The current implementation compares the robot's current grid cell with all remaining observation stations and selects the station with the minimum Manhattan distance. 

This process repeats after each unsuccessful observation until either:

- the target object is found; or
- all observation stations have been searched.

An alternative station-selection approach using actual path length was experimentally tested. However, it produced the same station visitation order and identical completion time in the tested Start C `soda_can` scenario. The simpler Manhattan-distance greedy approach was therefore retained.

---

## Path Planning

Once an observation station is selected, the system computes a route from the robot's current grid cell to the station.

The current path planner:

- searches across four grid directions;
- avoids cells marked as obstacles;
- assigns an equal movement cost of one per grid step; and
- uses a priority queue to determine the next cell to explore. 

The planner therefore operates as a uniform-cost/Dijkstra-style grid search.

---

## Movement and Replanning

The robot follows the direction labels returned by the path planner.

During forward movement, proximity sensors are continuously checked.

If a safety event occurs:

1. movement is stopped;
2. the detected hazard direction is returned to the main controller;
3. the current path traversal is interrupted; and
4. a new path is calculated from the robot's updated position.

This allows the navigation system to recover from unexpected proximity events without terminating the mission.

---

## Perception

The robot uses its onboard RGB camera to classify objects at each observation station.

The current integrated perception system uses a multi-view approach.

At each observation station, the robot:

1. aligns to the station's observation yaw;
2. rotates through multiple viewing angles;
3. captures multiple camera images;
4. runs the detector across those images; and
5. selects the best-confidence prediction.

If the predicted object does not match the mission target, the robot proceeds to the next observation station.

If the predicted object matches the mission target, the mission terminates successfully.

---

## Safety System

### Directional proximity monitoring

The safety system interprets the eight e-puck proximity sensors using directional groups:

- FRONT: `ps0` and `ps7` (straight ahead), plus front-diagonal `ps1` and `ps6`
- RIGHT: `ps2`
- BACK: `ps3` and `ps4`
- LEFT: `ps5`

The following empirically calibrated thresholds are currently used:

```python
FRONT_DANGER_THRESHOLD = 120.0
RIGHT_DANGER_THRESHOLD = 150.0
BACK_DANGER_THRESHOLD = 120.0
LEFT_DANGER_THRESHOLD = 150.0
```

These thresholds were selected through repeated Webots calibration runs rather than assumed from theoretical sensor values.

---

## Safety Behaviour

The safety system can return multiple simultaneous hazard directions.

Examples include:

```python
set()
```

for no danger,

```python
{ProximityDirection.FRONT}
```

for a frontal hazard, or

```python
{
    ProximityDirection.LEFT,
    ProximityDirection.RIGHT
}
```

when multiple directions are simultaneously unsafe.

This design avoids assuming that the opposite side of a detected obstacle must automatically be safe.

---

## Front Recovery

Automatic physical recovery is currently implemented only for a pure FRONT hazard.

When a FRONT danger is detected:

1. the robot stops;
2. the system checks that the rear direction is not also dangerous;
3. the robot performs a short reverse manoeuvre;
4. proximity sensors continue to be monitored during recovery; and
5. the robot stops again before returning control to the main controller.

LEFT, RIGHT, BACK, and multi-directional events currently stop movement and return control for replanning instead of triggering an automatic directional recovery.

This conservative approach was retained because full-mission testing did not show a repeated replanning loop that required additional side-recovery behaviour.

---

# Validation

## Full-Mission Testing

Full missions were tested using all three provided starting positions:

- Start A
- Start B
- Start C

Multiple mission targets were first tested across all three starting positions.

### `soda_can`

| Start | Completion Time | Result |
|---|---:|---|
| A | 3:03 | Success |
| B | 3:13 | Success |
| C | 3:44 | Success |

Start C was repeated using the same target and again completed in approximately 3:44, indicating that this runtime was reproducible rather than an isolated delay.

---

### `wall_clock`

| Start | Completion Time | Result |
|---|---:|---|
| A | 0:22 | Success |
| B | 0:41 | Success |
| C | 1:28 | Success |

The target appeared earlier in the station search order for these scenarios, resulting in substantially shorter completion times.

---

### `backpack`

| Start | Completion Time | Result |
|---|---:|---|
| A | 3:38 | Success |
| B | 2:37 | Success |
| C | 0:19 | Success |

Again, completion time depended primarily on how early the target station appeared in the search sequence.

---

## Remaining Target-Class Validation

The five remaining target classes were also tested as actual mission targets.

| Run | Start | Target | Completion Time | Detection Position | Result |
|---|---|---|---:|---:|---|
| 1 | A | `coffee_mug` | 2:37 | 6th | Success |
| 2 | B | `fire_extinguisher` | 1:20 | 4th | Success |
| 3 | C | `camera` | 2:58 | 7th | Success |
| 4 | A | `running_shoe` | 1:22 | 3rd | Success |
| 5 | B | `headphones` | 0:20 | 1st | Success |

All eight supported target classes were therefore successfully used as mission targets and correctly identified during autonomous search.

No incorrect early mission termination was observed in the recorded validation runs.

---

# Timing Analysis

The longest reproducible test was:

```text
Start C
Target: soda_can
Completion time: approximately 3:44
```

Detailed simulation-time logging was added temporarily to identify the main sources of runtime.

The total mission time was approximately:

```text
223.6 seconds
```

The approximate breakdown was:

| Component | Total Time | Approximate Share |
|---|---:|---:|
| Navigation | 139.1 s | 62% |
| Multi-view scanning | 58.9 s | 26% |
| Yaw alignment | 25.6 s | 11% |

The multi-view perception process required approximately:

```text
7.36 seconds per station
```

Across eight stations:

```text
7.36 × 8 = 58.88 seconds
```

The longest single navigation segment occurred before the final station and required approximately:

```text
40 seconds
```

This analysis indicated that navigation distance and station visitation order were the main contributors to the longest runtime rather than the safety system.

---

## Station-Selection Experiment

An experimental version of the station selector replaced Manhattan distance with the actual path length calculated by the path planner.

The intention was to determine whether Manhattan distance was selecting stations that appeared geographically close but required longer routes around obstacles.

For the Start C `soda_can` test:

```text
Original greedy selection: 3:44
Path-length-based selection: 3:44
```

The station visitation order also remained unchanged.

Because the alternative approach increased implementation complexity without improving runtime, the original greedy Manhattan-distance approach was retained.

---

# Safety Validation

## Normal Mission Behaviour

During the final normal mission validation runs, no unnecessary safety intervention occurred when the robot remained within safe navigation corridors.

This is expected behaviour because the safety layer is designed to intervene only when proximity readings exceed the configured thresholds.

---

## Deliberate FRONT Safety Stress Test

A separate safety integration test was created to deliberately move the robot toward an obstacle until the FRONT danger threshold was exceeded.

During the test:

- Moves 1–15 produced typical safe proximity readings of approximately 60–75.
- On Move 16, a FRONT safety event was detected.
- The robot immediately stopped forward movement.
- A short reverse recovery manoeuvre was executed.
- Recovery completed without collision.
- The FRONT proximity readings returned to the safe baseline range.
- The controller returned:

```text
SAFETY EVENT: ['FRONT']
```

After recovery, example front readings were approximately:

```text
ps0 ≈ 71.18
ps7 ≈ 64.73
```

Both were well below the configured FRONT danger threshold of:

```text
120.0
```

The test therefore confirmed the complete FRONT safety sequence:

```text
Forward movement
→ hazard detection
→ movement stop
→ reverse recovery
→ safe-distance restoration
→ safety event returned
```

---

# Validation Outcome

The current integrated system successfully demonstrated:

- autonomous mission execution;
- support for all three starting positions;
- successful mission targeting for all eight object classes;
- correct continuation after non-target detections;
- correct termination after target detection;
- navigation between all tested observation stations;
- multi-view visual object recognition;
- directional proximity-based safety detection;
- successful FRONT obstacle recovery;
- replanning support after safety events; and
- completion of all recorded full-mission validation runs within the four-minute simulation requirement.

The longest recorded successful mission was approximately:

```text
3 minutes 44 seconds
```

which remained within the four-minute requirement.

---
