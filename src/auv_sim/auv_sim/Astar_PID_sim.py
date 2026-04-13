#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt

from Astar import build_prequal_course, plan_multi_segment_route, grid_to_meters


# ============================================================
# HELPERS
# ============================================================
def grid_path_to_world(path_grid, meta):
    resolution = meta["resolution"]
    x_min = meta["x_min"]
    y_min = meta["y_min"]

    path_world = []
    for r, c in path_grid:
        x_m, y_m = grid_to_meters(r, c, x_min, y_min, resolution)
        path_world.append((x_m, y_m))
    return path_world


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def sample_path(path_world, step=0.4):
    if len(path_world) < 2:
        return path_world[:]

    sampled = [path_world[0]]
    last_x, last_y = path_world[0]

    for x, y in path_world[1:]:
        dist = math.hypot(x - last_x, y - last_y)
        if dist >= step:
            sampled.append((x, y))
            last_x, last_y = x, y

    if sampled[-1] != path_world[-1]:
        sampled.append(path_world[-1])

    return sampled

# PID SIMULATION
def simulate_auv_with_pid(
    path_world,
    mode="untuned",
    dt=0.1,
    max_steps=4000,
):
    """
    Simple 2D AUV simulation:
    state: x, y, heading
    controller: heading PID toward current path point
    disturbances: constant current + small random drift
    """

    if len(path_world) < 2:
        raise ValueError("Path must contain at least 2 points.")

    x = path_world[0][0]
    y = path_world[0][1]
    heading = 0.0

    actual_path = [(x, y)]

    current_x = 0.03
    current_y = -0.015

    rng = np.random.default_rng(7)

    if mode == "untuned":
        kp = 0.8
        ki = 0.0
        kd = 0.02
        heading_rate_limit = 0.55
        waypoint_tolerance = 0.45
        lookahead = 1
        speed_cmd = 0.75
    elif mode == "tuned":
        kp = 2.2
        ki = 0.03
        kd = 0.30
        heading_rate_limit = 1.2
        waypoint_tolerance = 0.60
        lookahead = 2
        speed_cmd = 0.95
    else:
        raise ValueError("mode must be 'untuned' or 'tuned'")

    integral_error = 0.0
    previous_error = 0.0
    target_index = 1

    for _ in range(max_steps):
        if target_index >= len(path_world):
            break

        tx, ty = path_world[target_index]

        dx = tx - x
        dy = ty - y
        distance_to_target = math.hypot(dx, dy)

        if distance_to_target < waypoint_tolerance:
            target_index += lookahead
            if target_index >= len(path_world):
                break
            continue

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(desired_heading - heading)

        integral_error += heading_error * dt
        derivative_error = (heading_error - previous_error) / dt
        previous_error = heading_error

        yaw_rate_cmd = (
            kp * heading_error
            + ki * integral_error
            + kd * derivative_error
        )

        yaw_rate_cmd = max(-heading_rate_limit, min(heading_rate_limit, yaw_rate_cmd))

        yaw_rate_actual = 0.90 * yaw_rate_cmd + rng.normal(0.0, 0.015)

        heading += yaw_rate_actual * dt
        heading = wrap_to_pi(heading)

        vx_body = speed_cmd * math.cos(heading)
        vy_body = speed_cmd * math.sin(heading)

        disturbance_x = rng.normal(0.0, 0.008)
        disturbance_y = rng.normal(0.0, 0.008)

        x += (vx_body + current_x + disturbance_x) * dt
        y += (vy_body + current_y + disturbance_y) * dt

        actual_path.append((x, y))

        gx, gy = path_world[-1]
        if math.hypot(gx - x, gy - y) < 0.5 and target_index >= len(path_world) - 2:
            break

    return actual_path

# PLOTTING
def plot_before_pid(meta, planned_path_world, actual_untuned):
    x_min = meta["x_min"]
    x_max = meta["x_max"]
    y_min = meta["y_min"]
    y_max = meta["y_max"]

    plt.figure(figsize=(14, 8))

    px = [p[0] for p in planned_path_world]
    py = [p[1] for p in planned_path_world]
    plt.plot(px, py, linewidth=3.0, color='cyan', label="Optimal A* Path")

    ux = [p[0] for p in actual_untuned]
    uy = [p[1] for p in actual_untuned]
    plt.plot(
        ux, uy,
        linewidth=3.0,
        linestyle='--',
        color='red',
        label="Actual Path (Before PID Tuning)"
    )

    sx, sy = meta["start_xy"]
    plt.scatter(sx, sy, s=160, marker="o", color='green', label="Start")

    lgx, lgy = meta["left_gate_post_xy"]
    rgx, rgy = meta["right_gate_post_xy"]
    plt.scatter([lgx, rgx], [lgy, rgy], s=130, marker="s", color='black', label="Gate")

    mx, my = meta["marker_center_xy"]
    plt.scatter(mx, my, s=180, marker="X", color='red', label="Marker")

    wpx = [p[0] for p in meta["waypoints_xy"]]
    wpy = [p[1] for p in meta["waypoints_xy"]]
    plt.scatter(wpx, wpy, s=70, marker="D", color='orange', label="Mission Waypoints")

    plt.title("A* Path Planning vs Actual Path Before PID Tuning", fontsize=28)
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
        labels.index("Actual Path (Before PID Tuning)"),
    ]

    plt.legend(
        [handles[i] for i in order],
        [labels[i] for i in order],
        loc='center left',
        bbox_to_anchor=(1.02, 0.5),
        fontsize=20
    )

    plt.tight_layout()
    plt.savefig("before_pid.png", dpi=300, bbox_inches='tight')
    plt.show()


def plot_after_pid(meta, planned_path_world, actual_tuned):
    x_min = meta["x_min"]
    x_max = meta["x_max"]
    y_min = meta["y_min"]
    y_max = meta["y_max"]

    plt.figure(figsize=(14, 8))

    px = [p[0] for p in planned_path_world]
    py = [p[1] for p in planned_path_world]
    plt.plot(px, py, linewidth=3.0, color='cyan', label="Optimal A* Path")

    tx = [p[0] for p in actual_tuned]
    ty = [p[1] for p in actual_tuned]
    plt.plot(
        tx, ty,
        linewidth=3.0,
        linestyle='--',
        color='blue',
        label="Actual Path (After PID Tuning)"
    )

    sx, sy = meta["start_xy"]
    plt.scatter(sx, sy, s=160, marker="o", color='green', label="Start")

    lgx, lgy = meta["left_gate_post_xy"]
    rgx, rgy = meta["right_gate_post_xy"]
    plt.scatter([lgx, rgx], [lgy, rgy], s=130, marker="s", color='black', label="Gate")

    mx, my = meta["marker_center_xy"]
    plt.scatter(mx, my, s=180, marker="X", color='red', label="Marker")

    wpx = [p[0] for p in meta["waypoints_xy"]]
    wpy = [p[1] for p in meta["waypoints_xy"]]
    plt.scatter(wpx, wpy, s=70, marker="D", color='orange', label="Mission Waypoints")

    plt.title("A* Path Planning vs Actual Path After PID Tuning", fontsize=28)
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
        labels.index("Actual Path (After PID Tuning)"),
    ]

    plt.legend(
        [handles[i] for i in order],
        [labels[i] for i in order],
        loc='center left',
        bbox_to_anchor=(1.02, 0.5),
        fontsize=20
    )

    plt.tight_layout()
    plt.savefig("after_pid.png", dpi=300, bbox_inches='tight')
    plt.show()

def main():
    grid, waypoint_list_grid, meta = build_prequal_course()
    path_grid, _, _ = plan_multi_segment_route(grid, waypoint_list_grid)

    planned_path_world = grid_path_to_world(path_grid, meta)
    planned_path_world = sample_path(planned_path_world, step=0.4)

    actual_untuned = simulate_auv_with_pid(planned_path_world, mode="untuned")
    actual_tuned = simulate_auv_with_pid(planned_path_world, mode="tuned")

    plot_before_pid(meta, planned_path_world, actual_untuned)
    plot_after_pid(meta, planned_path_world, actual_tuned)


if __name__ == "__main__":
    main()