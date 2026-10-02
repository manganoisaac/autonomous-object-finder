"""Render the greedy nearest-neighbour station plan on the occupancy grid."""

import argparse
import pathlib

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D

from path import Path
from project_utils import CONFIG, ROOT, world_to_grid
from stations import Stations

FREE_COLOR = "#ffffff"
OBSTACLE_COLOR = "#3a3a3a"
PATH_COLOR = "#1f6fb2"
VISITED_COLOR = "#e07b00"
TARGET_COLOR = "#c0272d"
UNVISITED_COLOR = "#9aa0a6"


def station_definitions():
    return [
        (st["observe"], st["observe_yaw"], st["id"]) for st in CONFIG["stations"]
    ]


def find_start(start_id):
    start = next(s for s in CONFIG["starts"] if s["id"] == start_id)
    return world_to_grid(start["pose"][0], start["pose"][1])


def greedy_order(start_cell, station_defs, target_id):
    remaining = Stations(list(station_defs))
    cell_to_id = {
        world_to_grid(observe[0], observe[1]): station_id
        for observe, _, station_id in station_defs
    }

    order = []
    current = start_cell
    while len(remaining) > 0:
        observe, _ = remaining.pop(current)
        station_id = cell_to_id[observe]
        order.append((station_id, observe))
        current = observe
        if station_id == target_id:
            break

    return order


def leg_cells(planner, start_cell, goal_cell):
    try:
        result = planner.path_a_star(start_cell, goal_cell)
    except Exception:
        return [start_cell, goal_cell]
    return [cell for cell, _ in result]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", default="A")
    parser.add_argument("--target-station", default="S4")
    parser.add_argument(
        "--output",
        default=str(pathlib.Path(__file__).with_name("route_planning.pdf")),
    )
    parser.add_argument("--dpi", type=int, default=200)
    args = parser.parse_args()

    grid = np.load(ROOT / "maps" / "occupancy_grid.npy")
    planner = Path(checkpoints=[], map=grid)

    station_defs = station_definitions()
    start_cell = find_start(args.start)
    order = greedy_order(start_cell, station_defs, args.target_station)

    visited_ids = {station_id for station_id, _ in order}
    route_cells = [start_cell] + [observe for _, observe in order]

    fig, ax = plt.subplots(figsize=(6.2, 6.2))
    ax.imshow(
        grid,
        cmap=ListedColormap([FREE_COLOR, OBSTACLE_COLOR]),
        origin="upper",
        interpolation="none",
        vmin=0,
        vmax=1,
    )

    for index, start in enumerate(route_cells[:-1]):
        cells = leg_cells(planner, start, route_cells[index + 1])
        columns = [col for _, col in cells]
        rows = [row for row, _ in cells]
        ax.plot(columns, rows, color=PATH_COLOR, linewidth=2.0, zorder=3)

        midpoint = len(cells) // 2
        tail = cells[max(midpoint - 1, 0)]
        head = cells[midpoint]
        if tail != head:
            ax.annotate(
                "",
                xy=(head[1], head[0]),
                xytext=(tail[1], tail[0]),
                arrowprops=dict(arrowstyle="-|>", color=PATH_COLOR, lw=1.6),
                zorder=4,
            )

    ax.scatter(
        start_cell[1],
        start_cell[0],
        marker="o",
        s=140,
        color="#2e7d32",
        edgecolor="white",
        linewidth=1.2,
        zorder=6,
    )
    ax.annotate(
        f"Start {args.start}",
        (start_cell[1], start_cell[0]),
        textcoords="offset points",
        xytext=(8, 6),
        fontsize=9,
        fontweight="bold",
        color="#2e7d32",
        zorder=7,
    )

    for rank, (station_id, observe) in enumerate(order, 1):
        is_target = station_id == args.target_station
        ax.scatter(
            observe[1],
            observe[0],
            marker="*" if is_target else "o",
            s=260 if is_target else 110,
            color=TARGET_COLOR if is_target else VISITED_COLOR,
            edgecolor="white",
            linewidth=1.2,
            zorder=6,
        )
        label = f"{station_id} ({rank})"
        if is_target:
            label = f"{station_id} (target)"
        ax.annotate(
            label,
            (observe[1], observe[0]),
            textcoords="offset points",
            xytext=(7, 5),
            fontsize=8.5,
            color=TARGET_COLOR if is_target else "#8a4b00",
            fontweight="bold" if is_target else "normal",
            zorder=7,
        )

    for observe, _, station_id in station_defs:
        if station_id in visited_ids:
            continue
        row, col = world_to_grid(observe[0], observe[1])
        ax.scatter(
            col,
            row,
            marker="o",
            s=70,
            facecolor="none",
            edgecolor=UNVISITED_COLOR,
            linewidth=1.2,
            zorder=5,
        )
        ax.annotate(
            station_id,
            (col, row),
            textcoords="offset points",
            xytext=(6, 4),
            fontsize=8,
            color=UNVISITED_COLOR,
            zorder=7,
        )

    order_text = " \u2192 ".join(["Start " + args.start] + [s for s, _ in order])
    ax.set_title(
        f"Greedy nearest-neighbour station order\n{order_text}",
        fontsize=10,
    )
    ax.set_xlabel("grid column")
    ax.set_ylabel("grid row")
    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)
    ax.set_xticks(np.arange(0, grid.shape[1], 5))
    ax.set_yticks(np.arange(0, grid.shape[0], 5))

    legend_handles = [
        Line2D([0], [0], color=PATH_COLOR, lw=2, label="A* route"),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor="#2e7d32",
            markeredgecolor="white", markersize=9, label="Start",
        ),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor=VISITED_COLOR,
            markeredgecolor="white", markersize=8, label="Visited station",
        ),
        Line2D(
            [0], [0], marker="*", color="none", markerfacecolor=TARGET_COLOR,
            markeredgecolor="white", markersize=12, label="Target station",
        ),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor="none",
            markeredgecolor=UNVISITED_COLOR, markersize=7, label="Unvisited",
        ),
    ]
    ax.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=3,
        fontsize=8,
        frameon=False,
    )

    output_path = pathlib.Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {output_path.resolve()}")
    print("Visit order:", order_text)


if __name__ == "__main__":
    main()
