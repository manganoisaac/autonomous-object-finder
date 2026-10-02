"""Benchmark the Dijkstra and A* planners in path/path.py."""

import argparse
import csv
import pathlib
import statistics
import time
from collections import deque

import numpy as np

from path import Path
from project_utils import CONFIG, ROOT, world_to_grid

MIN_CELL = 2
MAX_CELL = 38


def is_free(grid, cell):
    row, col = cell
    return (
        MIN_CELL <= row < MAX_CELL
        and MIN_CELL <= col < MAX_CELL
        and grid[row][col] == 0
    )


def nearest_free(grid, cell, max_radius=5):
    if is_free(grid, cell):
        return cell

    seen = {cell}
    queue = deque([(cell[0], cell[1], 0)])

    while queue:
        row, col, dist = queue.popleft()

        if is_free(grid, (row, col)):
            return (row, col)

        if dist >= max_radius:
            continue

        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            new_cell = (row + d_row, col + d_col)
            if new_cell not in seen:
                seen.add(new_cell)
                queue.append((new_cell[0], new_cell[1], dist + 1))

    return None


def build_points(grid, which):
    points = []
    used_cells = set()

    def add(label, world_x, world_y):
        cell = nearest_free(grid, world_to_grid(world_x, world_y))
        if cell is not None and cell not in used_cells:
            used_cells.add(cell)
            points.append((label, cell))

    if which in ("start", "all"):
        for start in CONFIG["starts"]:
            add(f"start_{start['id']}", start["pose"][0], start["pose"][1])

    if which in ("station", "all"):
        for station in CONFIG["stations"]:
            add(f"station_{station['id']}", station["observe"][0], station["observe"][1])

    return points


def timed_call(planner, start, goal, repeats, warmup):
    result = {"success": False, "path_len": None, "times": [], "error": ""}

    for _ in range(warmup):
        try:
            planner(start, goal)
        except Exception:
            break

    for _ in range(repeats):
        begin = time.perf_counter()
        try:
            path = planner(start, goal)
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            return result
        result["times"].append((time.perf_counter() - begin) * 1000.0)
        result["path_len"] = len(path)

    if result["times"]:
        result["success"] = True
    else:
        result["error"] = result["error"] or "no timings collected"

    return result


def summarise(times):
    return {
        "min": min(times),
        "mean": statistics.mean(times),
        "median": statistics.median(times),
        "max": max(times),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=25)
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument(
        "--points",
        choices=("start", "station", "all"),
        default="all",
    )
    parser.add_argument(
        "--output",
        default=str(pathlib.Path(__file__).with_name("path_benchmark_results.csv")),
    )
    args = parser.parse_args()

    grid = np.load(ROOT / "maps" / "occupancy_grid.npy")
    planner = Path(checkpoints=[], map=grid)

    algorithms = [
        ("dijkstra", planner.path),
        ("a_star", planner.path_a_star),
    ]

    points = build_points(grid, args.points)
    pairs = [
        (start_label, start_cell, goal_label, goal_cell)
        for start_label, start_cell in points
        for goal_label, goal_cell in points
        if start_cell != goal_cell
    ]

    print(
        f"Benchmarking {len(algorithms)} algorithms over {len(pairs)} pairs "
        f"({len(points)} points, repeats={args.repeats}, warmup={args.warmup})"
    )
    print(f"Map: {ROOT / 'maps' / 'occupancy_grid.npy'}")
    print("")

    rows = []
    path_lengths = {}

    for index, (start_label, start_cell, goal_label, goal_cell) in enumerate(pairs, 1):
        pair_id = index
        line_parts = []

        for name, planner_fn in algorithms:
            stats = timed_call(
                planner_fn, start_cell, goal_cell, args.repeats, args.warmup
            )
            times = stats["times"]
            summary = summarise(times) if times else None

            row = {
                "pair_id": pair_id,
                "algorithm": name,
                "start_label": start_label,
                "start_row": start_cell[0],
                "start_col": start_cell[1],
                "goal_label": goal_label,
                "goal_row": goal_cell[0],
                "goal_col": goal_cell[1],
                "success": stats["success"],
                "path_len": stats["path_len"],
                "repeats": len(times),
                "min_ms": f"{summary['min']:.6f}" if summary else "",
                "mean_ms": f"{summary['mean']:.6f}" if summary else "",
                "median_ms": f"{summary['median']:.6f}" if summary else "",
                "max_ms": f"{summary['max']:.6f}" if summary else "",
                "error": stats["error"],
            }
            rows.append(row)

            if summary:
                line_parts.append(
                    f"{name}: mean={summary['mean']:.4f}ms "
                    f"min={summary['min']:.4f}ms len={stats['path_len']}"
                )
                path_lengths.setdefault(pair_id, {})[name] = stats["path_len"]
            else:
                line_parts.append(f"{name}: FAILED ({stats['error']})")

        print(
            f"[{index:>3}/{len(pairs)}] "
            f"{start_label}{start_cell} -> {goal_label}{goal_cell}  "
            + "  |  ".join(line_parts)
        )

    output_path = pathlib.Path(args.output)
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print("")
    print("Summary")
    print("-------")

    overall = {}
    for name, _ in algorithms:
        times = [
            float(row["mean_ms"])
            for row in rows
            if row["algorithm"] == name and row["mean_ms"] != ""
        ]
        successes = sum(
            1 for row in rows if row["algorithm"] == name and row["success"]
        )
        if times:
            overall[name] = statistics.mean(times)
            print(
                f"{name:<9} mean-of-means={statistics.mean(times):.4f}ms "
                f"median-of-means={statistics.median(times):.4f}ms "
                f"fastest={min(times):.4f}ms slowest={max(times):.4f}ms "
                f"success={successes}/{len(pairs)}"
            )
        else:
            print(f"{name:<9} no successful runs (success={successes}/{len(pairs)})")

    if "dijkstra" in overall and "a_star" in overall and overall["a_star"] > 0:
        print(
            f"speedup  dijkstra/a_star = "
            f"{overall['dijkstra'] / overall['a_star']:.2f}x"
        )

    mismatches = [
        pair_id
        for pair_id, lengths in path_lengths.items()
        if len(set(lengths.values())) > 1
    ]
    print(f"path-length mismatches between algorithms: {len(mismatches)}")

    print("")
    print(f"CSV written to {output_path.resolve()}")


if __name__ == "__main__":
    main()
