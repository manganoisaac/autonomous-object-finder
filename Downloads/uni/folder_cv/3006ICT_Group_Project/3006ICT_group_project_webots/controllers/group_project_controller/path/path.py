import enum
import heapq
from dataclasses import dataclass
from typing import TypedDict


# class GLOBAL_MOVEMENT(enum.Enum):
#     UP = enum.auto()
#     DOWN = enum.auto()
#     LEFT = enum.auto()
#     RIGHT = enum.auto()
#
#
class COMMAND(enum.Enum):
    MOVE = enum.auto()
    TURN_LEFT = enum.auto()
    TURN_RIGHT = enum.auto()


#
# direction_weights = {
#     0: {(0, 1): "F", (1, 0): "R", (-1, 0): "L", (0, -1): "B"},
#     1: {(-1, 0): "F", (0, 1): "R", (0, -1): "L", (1, 0): "B"},
#     -2: {(0, -1): "F", (1, 0): "L", (-1, 0): "R", (0, 1): "B"},
#     -1: {(1, 0): "F", (0, 1): "L", (0, -1): "R", (-1, 0): "B"},
# }
#


@dataclass
class Frontier_Node:
    coord: tuple[int, int]
    direction: int
    path_len: int

    def __lt__(self, other):
        return False


tuple_to_dir_map = {(0, 1): 0, (-1, 0): 1, (0, -1): -2, (1, 0): -1}


@dataclass(frozen=True)
class Path:
    checkpoints: list[tuple[float, float, float]]
    map: list[list[int]]

    def path(self, start_cell: tuple[int, int], goal: tuple[int, int]) -> list[str]:
        # directions = {"UP": (0, 1), "DOWN": (0, -1), "RIGHT": (1, 0), "LEFT": (-1, 0)}
        # TODO: check (which one?)
        directions = {"UP": (-1, 0), "DOWN": (1, 0), "RIGHT": (0, 1), "LEFT": (0, -1)}
        frontier = [(0, start_cell, "")]
        routes = {}
        visited = set()
        visited.add(start_cell)
        while len(frontier) >= 0:
            cur_weight, cur_node, cur_direction = heapq.heappop(frontier)
            if cur_node == goal:
                res_path = [(goal, "")]
                cur = routes[goal]
                while cur is not None:
                    cur_node, dir_label = cur
                    res_path.append((cur_node, dir_label))
                    cur = routes.get(cur_node)
                res_path.reverse()
                return res_path

            for dir_label, dir_tuple in directions.items():
                new_cell = cur_node[0] + dir_tuple[0], cur_node[1] + dir_tuple[1]
                if (
                    new_cell in visited
                    or new_cell[0] < 2
                    or new_cell[0] >= 38
                    or new_cell[1] < 2
                    or new_cell[1] >= 38
                    # TODO: check x,y or y,x?
                    or self.map[new_cell[0]][new_cell[1]] == 1
                ):
                    continue
                visited.add(new_cell)
                routes[new_cell] = cur_node, dir_label
                heapq.heappush(frontier, (cur_weight + 1, new_cell, dir_label))
