#!/usr/bin/env python3

import heapq
import math
import matplotlib.pyplot as plt
import numpy as np


# ============================================================
# A* NODE
# ============================================================
class Node:
    def __init__(self, position, parent=None):
        self.position = position   # (row, col)
        self.parent = parent
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0

    def __lt__(self, other):
        return self.f < other.f


# ============================================================
# A* HELPERS
# ============================================================
def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def reconstruct_path(current_node):
    path = []
    while current_node is not None:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]


def astar_with_tree(grid, start, goal):
    rows, cols = grid.shape

    open_list = []
    closed_set = set()
    g_scores = {}

    explored_nodes = []
    expansion_edges = []

    start_node = Node(start)
    goal_node = Node(goal)

    start_node.g = 0.0
    start_node.h = heuristic(start, goal)
    start_node.f = start_node.g + start_node.h

    heapq.heappush(open_list, start_node)
    g_scores[start] = 0.0

    neighbors = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position in closed_set:
            continue

        closed_set.add(current_node.position)
        explored_nodes.append(current_node.position)

        if current_node.position == goal_node.position:
            return reconstruct_path(current_node), explored_nodes, expansion_edges

        for dr, dc in neighbors:
            nr = current_node.position[0] + dr
            nc = current_node.position[1] + dc
            neighbor_pos = (nr, nc)

            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue

            if grid[nr, nc] == 1:
                continue

            if neighbor_pos in closed_set:
                continue

            step_cost = math.sqrt(2) if (dr != 0 and dc != 0) else 1.0
            tentative_g = current_node.g + step_cost

            if neighbor_pos not in g_scores or tentative_g < g_scores[neighbor_pos]:
                g_scores[neighbor_pos] = tentative_g

                neighbor_node = Node(neighbor_pos, current_node)
                neighbor_node.g = tentative_g
                neighbor_node.h = heuristic(neighbor_pos, goal_node.position)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                heapq.heappush(open_list, neighbor_node)
                expansion_edges.append((current_node.position, neighbor_pos))

    return None, explored_nodes, expansion_edges


# ============================================================
# WORLD / GRID CONVERSION
# ============================================================
def meters_to_grid(x_m, y_m, x_min, y_min, resolution):
    col = int(round((x_m - x_min) / resolution))
    row = int(round((y_m - y_min) / resolution))
    return row, col


def grid_to_meters(row, col, x_min, y_min, resolution):
    x_m = x_min + col * resolution
    y_m = y_min + row * resolution
    return x_m, y_m


def add_circular_obstacle(grid, center_xy_m, radius_m, x_min, y_min, resolution):
    rows, cols = grid.shape
    cx, cy = center_xy_m

    for r in range(rows):
        for c in range(cols):
            x, y = grid_to_meters(r, c, x_min, y_min, resolution)
            if math.sqrt((x - cx)**2 + (y - cy)**2) <= radius_m:
                grid[r, c] = 1


def add_rect_obstacle(grid, center_xy_m, width_m, height_m, x_min, y_min, resolution):
    cx, cy = center_xy_m
    half_w = width_m / 2.0
    half_h = height_m / 2.0

    x0 = cx - half_w
    x1 = cx + half_w
    y0 = cy - half_h
    y1 = cy + half_h

    rows, cols = grid.shape
    for r in range(rows):
        for c in range(cols):
            x, y = grid_to_meters(r, c, x_min, y_min, resolution)
            if x0 <= x <= x1 and y0 <= y <= y1:
                grid[r, c] = 1


# ============================================================
# BUILD COURSE
# ============================================================
def build_prequal_course():
    resolution = 0.20

    # Original-style bounds that produced your preferred figure
    x_min, x_max = 0.0, 25.0
    y_min, y_max = 0.0, 12.5

    cols = int(round((x_max - x_min) / resolution)) + 1
    rows = int(round((y_max - y_min) / resolution)) + 1

    grid = np.zeros((rows, cols), dtype=int)

    # Start, gate, marker in original-style layout
    start_xy = (1.0, 6.25)
    gate_center_xy = (5, 6.25)
    marker_center_xy = (15.0, 6.25)

    # Gate posts: these gave the vertical gate appearance in the original graph
    left_gate_post_xy = (5, 5.25)
    right_gate_post_xy = (5, 7.25)
    gate_post_radius = 0.18

    marker_radius = 0.35

    add_circular_obstacle(grid, left_gate_post_xy, gate_post_radius, x_min, y_min, resolution)
    add_circular_obstacle(grid, right_gate_post_xy, gate_post_radius, x_min, y_min, resolution)
    add_circular_obstacle(grid, marker_center_xy, marker_radius, x_min, y_min, resolution)

    # Waypoints 
    wp1_xy = (5.0, 6.25)
    wp2_xy = (15.0, 2.0)
    wp3_xy = (20, 6.25)
    wp4_xy = (15.0, 10)
    wp5_xy = (12, 6.25)
    wp6_xy = (3.0, 6.25)

    planning_points = [
        start_xy,
        wp1_xy,
        wp2_xy,
        wp3_xy,
        wp4_xy,
        wp5_xy,
        wp6_xy,
    ]
    
    waypoint_list_xy = [
        wp1_xy,
        wp2_xy,
        wp3_xy,
        wp4_xy,
        wp5_xy,
        wp6_xy
    ]

    waypoint_list_grid = [
        meters_to_grid(x, y, x_min, y_min, resolution)
        for (x, y) in planning_points
    ]

    meta = {
        "resolution": resolution,
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "start_xy": start_xy,
        "gate_center_xy": gate_center_xy,
        "marker_center_xy": marker_center_xy,
        "left_gate_post_xy": left_gate_post_xy,
        "right_gate_post_xy": right_gate_post_xy,
        "waypoints_xy": waypoint_list_xy
    }

    return grid, waypoint_list_grid, meta


# ============================================================
# PLAN MULTI-SEGMENT ROUTE
# ============================================================
def plan_multi_segment_route(grid, waypoint_list_grid):
    full_path = []
    all_explored = []
    all_edges = []

    for i in range(len(waypoint_list_grid) - 1):
        start = waypoint_list_grid[i]
        goal = waypoint_list_grid[i + 1]

        segment_path, explored_nodes, expansion_edges = astar_with_tree(grid, start, goal)

        if segment_path is None:
            raise RuntimeError(f"A* failed between waypoint {i} and waypoint {i+1}")

        if i > 0:
            segment_path = segment_path[1:]

        full_path.extend(segment_path)
        all_explored.extend(explored_nodes)
        all_edges.extend(expansion_edges)

    return full_path, all_explored, all_edges


# ============================================================
# PLOT
# ============================================================
def plot_prequal_astar(grid, path, explored_nodes, expansion_edges, meta):
    resolution = meta["resolution"]
    x_min = meta["x_min"]
    x_max = meta["x_max"]
    y_min = meta["y_min"]
    y_max = meta["y_max"]

    plt.figure(figsize=(14, 8))

    plt.imshow(
        grid,
        cmap="Greys",
        origin="lower",
        extent=[x_min, x_max, y_min, y_max],
        alpha=0.35
    )

    for parent, child in expansion_edges:
        px, py = grid_to_meters(parent[0], parent[1], x_min, y_min, resolution)
        cx, cy = grid_to_meters(child[0], child[1], x_min, y_min, resolution)
        plt.plot([px, cx], [py, cy], linewidth=0.5, alpha=0.18)

    if explored_nodes:
        ex = []
        ey = []
        for r, c in explored_nodes:
            x, y = grid_to_meters(r, c, x_min, y_min, resolution)
            ex.append(x)
            ey.append(y)
        plt.scatter(ex, ey, s=10, alpha=0.5, label="Explored Nodes")

    if path:
        px = []
        py = []
        for r, c in path:
            x, y = grid_to_meters(r, c, x_min, y_min, resolution)
            px.append(x)
            py.append(y)
        plt.plot(px, py, linewidth=3.0, color='cyan', label="Optimal A* Path")

    sx, sy = meta["start_xy"]
    plt.scatter(sx, sy, s=160, marker="o", color='green', label="Start")

    lgx, lgy = meta["left_gate_post_xy"]
    rgx, rgy = meta["right_gate_post_xy"]
    plt.scatter([lgx, rgx], [lgy, rgy], s=130, marker="s", color='black',label="Gate")

    mx, my = meta["marker_center_xy"]
    plt.scatter(mx, my, s=180, marker="X", color='red', label="Marker")

    wpx = [p[0] for p in meta["waypoints_xy"]]
    wpy = [p[1] for p in meta["waypoints_xy"]]
    plt.scatter(wpx, wpy, s=70, marker="D", color='orange', label="Mission Waypoints")

    plt.title("RoboSub Pre-Qualification Course: A* Search ", fontsize=28)
    plt.xlabel("Pool Length (m)", fontsize=20)
    plt.ylabel("Pool Width (m)", fontsize=20)
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.grid(True, alpha=0.3)
    handles, labels = plt.gca().get_legend_handles_labels()

    order = [
        labels.index("Start"),
        labels.index("Gate"),
        labels.index("Marker"),
        labels.index("Mission Waypoints"),
        labels.index("Optimal A* Path"),
        labels.index("Explored Nodes"),
    ]

    plt.legend(
        [handles[i] for i in order],
        [labels[i] for i in order],
        loc='center left',
        bbox_to_anchor=(1.02, 0.5),
        fontsize=20
    )
    plt.tight_layout()
    plt.savefig("astar_plot.png", dpi=300, bbox_inches='tight')
    plt.show()


def main():
    grid, waypoint_list_grid, meta = build_prequal_course()
    path, explored_nodes, expansion_edges = plan_multi_segment_route(grid, waypoint_list_grid)
    plot_prequal_astar(grid, path, explored_nodes, expansion_edges, meta)


if __name__ == "__main__":
    main()