"""
Directional proximity safety checks for the e-puck robot. 
"""

from enum import Enum, auto

class ProximityDirection(Enum):
    FRONT = auto()
    RIGHT = auto()
    BACK = auto()
    LEFT = auto()

# initial danger threshold calibrated through webots proximity-sensor testing.
FRONT_DANGER_THRESHOLD = 120.0
RIGHT_DANGER_THRESHOLD = 150.0
BACK_DANGER_THRESHOLD = 120.0
LEFT_DANGER_THRESHOLD = 150.0


def check_proximity(proximity_values) -> set[ProximityDirection]:
    '''
    return all directional in which the robot is dangerously close to an obstacle

    Parameters 
    ----------
    proximity_values: list[float]
        Eight proximity sensor readings ordered from ps0 to ps7.

    returns
    -------
     set[ProximityDirection]
        The directions currently considered dangerous.
        An empty set indicates no detected danger.

    Raises
    ------
    ValueError
        If exactly eight proximity sensor readings are not provided.
    '''
    if len(proximity_values) != 8:
        raise ValueError("Expected 8 proximity sensor readings.")

    ps0, ps1, ps2, ps3, ps4, ps5, ps6, ps7 = proximity_values

    dangers = set()

    # front sensors 
    if max(ps0, ps7) > FRONT_DANGER_THRESHOLD:
        dangers.add(ProximityDirection.FRONT)

    # right sensors
    if ps2 > RIGHT_DANGER_THRESHOLD:
        dangers.add(ProximityDirection.RIGHT)

    # back sensors
    if max(ps3, ps4) > BACK_DANGER_THRESHOLD:
        dangers.add(ProximityDirection.BACK)

    # left sensor
    if ps5 > LEFT_DANGER_THRESHOLD:
        dangers.add(ProximityDirection.LEFT)

    return dangers


def is_dangerously_close(proximity_values):
        """
    Return True if at least one dangerous direction is detected.

    Retained for compatibility with existing controller code.
    """
        
        return bool(check_proximity(proximity_values))

