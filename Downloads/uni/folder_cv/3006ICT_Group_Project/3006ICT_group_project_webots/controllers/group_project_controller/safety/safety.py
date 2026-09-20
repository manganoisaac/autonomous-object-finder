# Development notes:
# 1. The threshold is defined as a constant so it can be easily adjusted
#    (e.g., 80 -> 78) without modifying the function itself.
# 2. The function checks that exactly eight proximity sensor values are provided,
#    helping us catch unexpected inputs early during integration.
# 3. The test block at the bottom allows the file to be run independently,
#    so Isaac or Tom can quickly verify the expected output:
#       Safe example: False
#       Danger example: True
#
# NOTE: These development notes and the test block will be removed before the final submission.

FRONT_DANGER_THRESHOLD = 80.0

def is_dangerously_close(proximity_values):
    """
    Return True when the robot is dangerously close to an obstacle
    in front of it.

    Parameters
    ----------
    proximity_values : list[float]
        Eight proximity sensor readings ordered from ps0 to ps7.

    Returns
    -------
    bool
        True if either front proximity sensor exceeds the danger
        threshold, otherwise False.

    Raises
    ------
    ValueError
        If exactly eight proximity sensor readings are not provided.
    """

    if len(proximity_values) != 8:
        raise ValueError("Expected 8 proximity sensor readings.")

    front_sensor_0 = proximity_values[0]
    front_sensor_7 = proximity_values[7]

    return max(front_sensor_0, front_sensor_7) > FRONT_DANGER_THRESHOLD


if __name__ == "__main__":
    safe_example = [63.7, 69.7, 63.8, 68.3, 61.3, 66.8, 71.5, 70.0]
    danger_example = [92.4, 62.2, 65.1, 69.5, 66.8, 67.5, 68.2, 86.9]

    print("Safe example:", is_dangerously_close(safe_example))
    print("Danger example:", is_dangerously_close(danger_example))