import enum
import heapq
from dataclasses import dataclass
from typing import TypedDict


class COMMAND(enum.Enum):
    MOVE = enum.auto()
    TURN_LEFT = enum.auto()
    TURN_RIGHT = enum.auto()


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

    def _m_dist(self, node: tuple[int, int], goal: tuple[int, int]) -> int:
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    # Djikstra to get shortest path to goal
    def path_a_star(
        self, start_cell: tuple[int, int], goal: tuple[int, int]
    ) -> list[str]:
        # Directions to iterate through
        # =============================
        directions = {"UP": (-1, 0), "DOWN": (1, 0), "RIGHT": (0, 1), "LEFT": (0, -1)}

        # Initialise frontier with tuples [path_dist, cell_coords, direction_label]
        # =========================================================================
        f_dist = self._m_dist(start_cell, goal)
        frontier = [(f_dist, 0, start_cell, "")]

        # Initialise routes dict: [cell: parent_cell]
        # ===========================================
        routes = {}

        # Initialise visited set [coord]
        # ==============================
        visited = set()
        visited.add(start_cell)

        # Search until frontier is empty or goal found
        # ============================================
        while len(frontier) >= 0:

            # Process the node with lowest path cost in frontier
            # ==================================================
            _, cur_weight, cur_node, cur_direction = heapq.heappop(frontier)

            # Base case: node is goal coordinate
            # ==================================
            if cur_node == goal:
                res_path = [(goal, "")]
                cur = routes[goal]
                while cur is not None:
                    cur_node, dir_label = cur
                    res_path.append((cur_node, dir_label))
                    cur = routes.get(cur_node)
                res_path.reverse()
                return res_path

            # Cycle through all four directions: up, down, left and right
            # ===========================================================
            for dir_label, dir_tuple in directions.items():

                # Vector addition to get new cell
                # ===============================
                new_cell = cur_node[0] + dir_tuple[0], cur_node[1] + dir_tuple[1]

                # Discard if not legitamite cell
                # ===============================
                if (
                    new_cell in visited
                    or new_cell[0] < 2
                    or new_cell[0] >= 38
                    or new_cell[1] < 2
                    or new_cell[1] >= 38
                    or self.map[new_cell[0]][new_cell[1]] == 1
                ):
                    continue

                # Add cell to visited
                # ===================
                visited.add(new_cell)

                # Set route for node with parent as current node
                # ==============================================
                routes[new_cell] = cur_node, dir_label

                f_dist = self._m_dist(new_cell, goal)
                new_weight = cur_weight + 1 + f_dist

                # Push node to heap with incremented weight and direction label
                # =============================================================
                heapq.heappush(frontier, (f_dist, new_weight, new_cell, dir_label))

    # Djikstra to get shortest path to goal
    def path(self, start_cell: tuple[int, int], goal: tuple[int, int]) -> list[str]:
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
                    or self.map[new_cell[0]][new_cell[1]] == 1
                ):
                    continue
                visited.add(new_cell)
                routes[new_cell] = cur_node, dir_label
                heapq.heappush(frontier, (cur_weight + 1, new_cell, dir_label))
