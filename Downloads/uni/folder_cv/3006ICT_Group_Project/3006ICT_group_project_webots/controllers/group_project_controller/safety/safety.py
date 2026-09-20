FRONT_DANGER_THRESHOLD = 80.0


def is_dangerously_close(proximity_values):
    """
    Check whether the robot is dangerously close to an obstacle
    in front of it.

    Parameters
    ----------
    proximity_values : list[float]
        Eight proximity sensor readings ordered from ps0 to ps7.

    Returns
    -------
    bool
        True if either front proximity sensor exceeds the
        danger threshold, otherwise False.
    """

    front_sensor_0 = proximity_values[0]
    front_sensor_7 = proximity_values[7]

    return max(front_sensor_0, front_sensor_7) > FRONT_DANGER_THRESHOLD

if __name__ == "__main__":
    safe_example = [63.7, 69.7, 63.8, 68.3, 61.3, 66.8, 71.5, 70.0]

    danger_example = [92.4, 62.2, 65.1, 69.5, 66.8, 67.5, 68.2, 86.9]

    print("Safe example:", is_dangerously_close(safe_example))
    print("Danger example:", is_dangerously_close(danger_example))