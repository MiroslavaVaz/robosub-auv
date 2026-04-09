#!/usr/bin/env python3

import math
import heapq

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header


class AStarPlannerNode(Node):
  
    def __init__(self):
        super().__init__('path_planner_node')

        # Grid / map settings
        self.resolution = 0.5  
        self.origin_x = -10.0
        self.origin_y = -5.0
        self.grid_width = int(20.0 / self.resolution)   
        self.grid_height = int(10.0 / self.resolution) 

        # 0 = free, 1 = obstacle
        self.occupancy_grid = [
            [0 for _ in range(self.grid_width)]
            for _ in range(self.grid_height)
        ]
     
        # Path mode
        # Set this to True for square path 
        self.use_square_test_path = True

        # Square test path in world coordinates
        self.square_waypoints = [
            (-7.8, 2.25),   # start
            (-7.0, 4.0),    # landmark 1
            (-6.0, 4.3),    # landmark 2
            (-4.5, 2.25),   # landmark 3
            (-7.8, 2.25),   # return to start/end
        ]

        # Multi-leg landmark route from your previous version
        self.start = (-8.0, 0.0)
        self.landmarks = [
            (-2.0,  2.0),
            ( 2.0, -2.0),
            ( 6.0,  1.0),
        ]

        self.define_obstacles()

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        self.get_logger().info('A* planner node started.')

    # Map helper functions

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.resolution + self.origin_x + self.resolution / 2.0
        y = gy * self.resolution + self.origin_y + self.resolution / 2.0
        return x, y

    def is_in_bounds(self, gx, gy):
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    def is_free(self, gx, gy):
        return self.is_in_bounds(gx, gy) and self.occupancy_grid[gy][gx] == 0

    def mark_rectangle(self, x_min, x_max, y_min, y_max):
        """Mark a rectangle in world coordinates as occupied."""
        gx_min, gy_min = self.world_to_grid(x_min, y_min)
        gx_max, gy_max = self.world_to_grid(x_max, y_max)

        gx_min = max(0, min(self.grid_width - 1, gx_min))
        gx_max = max(0, min(self.grid_width - 1, gx_max))
        gy_min = max(0, min(self.grid_height - 1, gy_min))
        gy_max = max(0, min(self.grid_height - 1, gy_max))

        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                self.occupancy_grid[gy][gx] = 1

    def define_obstacles(self):
        """
        Preliminary obstacles. Keep simple for testing.
        Adjust or remove these later to match your simulation world.
        """
        # Gate posts
        self.mark_rectangle(-9.2, -8.8, -1.3, -0.7)
        self.mark_rectangle(-9.2, -8.8,  0.7,  1.3)

        # Channel bars
        self.mark_rectangle(-5.0, -3.0, -0.3, 0.3)
        self.mark_rectangle(-1.0,  1.0, -0.3, 0.3)
        self.mark_rectangle( 3.0,  5.0, -0.3, 0.3)

    # ============================================================
    # A* planner functions
    # ============================================================

    def neighbors(self, node):
        x, y = node

        candidates = [
            (x + 1, y), (x - 1, y),
            (x, y + 1), (x, y - 1),
            (x + 1, y + 1), (x - 1, y - 1),
            (x + 1, y - 1), (x - 1, y + 1),
        ]

        valid = []
        for nx, ny in candidates:
            if self.is_free(nx, ny):
                valid.append((nx, ny))

        return valid

    @staticmethod
    def heuristic(a, b):
        x1, y1 = a
        x2, y2 = b
        return math.hypot(x2 - x1, y2 - y1)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star(self, start, goal):
        """
        Standard A* on a 2D grid.
        Returns a list of grid cells from start to goal, or [] if no path exists.
        """
        if not self.is_free(*start):
            self.get_logger().warn(f'Start cell {start} is blocked or out of bounds.')
            return []

        if not self.is_free(*goal):
            self.get_logger().warn(f'Goal cell {goal} is blocked or out of bounds.')
            return []

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed_set = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed_set:
                continue

            if current == goal:
                return self.reconstruct_path(came_from, current)

            closed_set.add(current)

            for nbr in self.neighbors(current):
                if nbr in closed_set:
                    continue

                tentative_g = g_score[current] + self.heuristic(current, nbr)

                if nbr not in g_score or tentative_g < g_score[nbr]:
                    came_from[nbr] = current
                    g_score[nbr] = tentative_g
                    f_score = tentative_g + self.heuristic(nbr, goal)
                    heapq.heappush(open_heap, (f_score, nbr))

        return []

    # ============================================================
    # Path generation / publishing
    # ============================================================

    def build_waypoint_list(self):
        """
        Returns world-coordinate waypoints for the route.
        """
        if self.use_square_test_path:
            return self.square_waypoints
        else:
            return [self.start] + self.landmarks + [self.start]

    def publish_path_from_waypoints(self, waypoints_world):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for wx, wy in waypoints_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Published clean waypoint path with {len(path_msg.poses)} points.'
        )

   # ROS callback
    def goal_callback(self, msg):
        """
        For now, the /goal message is only used as a trigger.
        Later, this can use msg.pose.position as the true goal.
        """
        self.get_logger().info('Goal trigger received. Planning route...')
        waypoints_world = self.build_waypoint_list()
        self.publish_path_from_waypoints(waypoints_world)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()