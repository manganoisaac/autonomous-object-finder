import enum
import heapq
from dataclasses import dataclass
from typing import TypedDict


class COMMAND(enum.Enum):
    MOVE = enum.auto()
    TURN_LEFT = enum.auto()
    TURN_RIGHT = enum.auto()


direction_weights = {
    0: {(0, 1): "F", (1, 0): "R", (-1, 0): "L", (0, -1): "B"},
    1: {(-1, 0): "F", (0, 1): "R", (0, -1): "L", (1, 0): "B"},
    -2: {(0, -1): "F", (1, 0): "L", (-1, 0): "R", (0, 1): "B"},
    -1: {(1, 0): "F", (0, 1): "L", (0, -1): "R", (-1, 0): "B"},
}


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

    # TODO: make direction an enum
    def path(self, cell: tuple[int, int], direction: int) -> list[COMMAND]:
        start_cell, start_direction = cell, direction
        print("step 1", flush=True)
        route_map = {}

        # initialise frontier
        counter = 0
        frontier: list[tuple[int, int, Frontier_Node]] = [
            (0, counter, Frontier_Node(coord=cell, direction=direction, path_len=0))
        ]
        counter += 1
        visited = set()

        goal = 37, 37
        i = 0

        while len(frontier) > 0 and i < 10000:
            i += 1
            print("step 2", flush=True)
            _, _, cur_node = heapq.heappop(frontier)
            print(f"at node: {cur_node}", flush=True)
            coord, direction, path_len = (
                cur_node.coord,
                cur_node.direction,
                cur_node.path_len,
            )
            if (coord, direction) in visited:
                continue
            visited.add((coord, direction))

            # check goal state
            if coord == goal:
                print(f"reached goal cell")
                break

            dir_map = direction_weights[direction]
            for m_coord, dir_str in dir_map.items():
                # TODO: something wrong here?
                next_cell = (coord[0] + m_coord[0], coord[1] + m_coord[1])

                # don't move if the block is a barrier
                if (
                    self.map[next_cell[0]][next_cell[1]] == 1
                    or next_cell[0] < 0
                    or next_cell[1] < 0
                    or next_cell[1] >= 40
                    or next_cell[0] >= 40
                ):
                    continue

                # push new node to frontier
                match dir_str:
                    case "F":
                        if (next_cell, direction) in visited:
                            continue
                        heapq.heappush(
                            frontier,
                            (
                                path_len + 1,
                                counter,
                                Frontier_Node(
                                    coord=next_cell,
                                    direction=direction,
                                    path_len=path_len + 1,
                                ),
                            ),
                        )
                        route_map[(next_cell, direction)] = (coord, direction)
                    case "L":
                        if (
                            next_cell,
                            tuple_to_dir_map[(-m_coord[1], m_coord[0])],
                        ) in visited:
                            continue
                        heapq.heappush(
                            frontier,
                            (
                                path_len + 2,
                                counter,
                                Frontier_Node(
                                    coord=next_cell,
                                    direction=tuple_to_dir_map[
                                        (-m_coord[1], m_coord[0])
                                    ],
                                    path_len=path_len + 2,
                                ),
                            ),
                        ),
                        route_map[
                            (next_cell, tuple_to_dir_map[(-m_coord[1], m_coord[0])])
                        ] = (coord, direction)
                    case "R":
                        if (
                            next_cell,
                            tuple_to_dir_map[(m_coord[1], -m_coord[0])],
                        ) in visited:
                            continue
                        heapq.heappush(
                            frontier,
                            (
                                path_len + 2,
                                counter,
                                Frontier_Node(
                                    coord=next_cell,
                                    direction=tuple_to_dir_map[
                                        (m_coord[1], -m_coord[0])
                                    ],
                                    path_len=path_len + 2,
                                ),
                            ),
                        )
                        route_map[
                            (next_cell, tuple_to_dir_map[(m_coord[1], -m_coord[0])])
                        ] = (coord, direction)
                    case "B":
                        if (
                            next_cell,
                            tuple_to_dir_map[(-m_coord[0], -m_coord[1])],
                        ) in visited:
                            continue
                        heapq.heappush(
                            frontier,
                            (
                                path_len + 3,
                                counter,
                                Frontier_Node(
                                    coord=next_cell,
                                    direction=tuple_to_dir_map[
                                        (-m_coord[0], -m_coord[1])
                                    ],
                                    path_len=path_len + 3,
                                ),
                            ),
                        )
                        route_map[
                            (next_cell, tuple_to_dir_map[(-m_coord[0], -m_coord[1])])
                        ] = (coord, direction)
                counter += 1

        node = route_map[(coord, direction)]
        i = 0
        route = []
        while node is not None and i < 1000 and node != (start_cell, start_direction):
            node = route_map[node]
            i += 1
            cur_coord, cur_direction = node
            route.append(node)
            print(f"node: {node}")

        route.reverse()
        print(f"final route", route)

        # run A*
        return [
            COMMAND.MOVE,
            COMMAND.TURN_LEFT,
            COMMAND.MOVE,
            COMMAND.TURN_RIGHT,
            COMMAND.MOVE,
            COMMAND.TURN_LEFT,
            COMMAND.MOVE,
            COMMAND.TURN_LEFT,
            COMMAND.MOVE,
            COMMAND.TURN_LEFT,
            COMMAND.MOVE,
            COMMAND.TURN_RIGHT,
            COMMAND.MOVE,
        ]
