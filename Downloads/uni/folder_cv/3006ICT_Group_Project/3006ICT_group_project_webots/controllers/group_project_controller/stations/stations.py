from dataclasses import dataclass
from typing import TypedDict

from project_utils import get_cell


class Station(TypedDict):
    observe: tuple[int, int]
    observe_yaw: float


def manhattan_dist(cell1, cell2) -> int:
    return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])


@dataclass()
class Stations:
    stations: dict[str, Station]

    def __init__(self, stations: list[tuple[tuple[float, float], float, str]]):
        """
        Initialises the stations into a list.
        Returns -> list[tuple[observation, yaw, id]]
        """
        self.stations = {}
        for observation, yaw, id in stations:
            station = Station(
                observe=get_cell(observation[0], observation[1]), observe_yaw=yaw
            )
            self.stations[id] = station

    def pop(self, robot_cell) -> tuple[tuple[int, int], float]:
        """
        Removes and returns the station with the minimum manhattan distance
        from the robot. This becomes the next point the robot moves to.
        """
        id, stn = min(
            self.stations.items(),
            key=lambda s: manhattan_dist(s[1]["observe"], robot_cell),
        )
        del self.stations[id]
        return stn["observe"], stn["observe_yaw"]

    def __len__(self) -> int:
        """
        Override the len() function with the length of the stations list
        """
        return len(self.stations)
