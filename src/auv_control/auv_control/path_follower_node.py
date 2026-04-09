#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        self.declare_parameter('waypoint_tolerance', 0.10)
        self.declare_parameter('cruise_speed', 0.18)
        self.declare_parameter('target_depth', 1.0)
        self.declare_parameter('final_tolerance', 0.12)

        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.target_depth = float(self.get_parameter('target_depth').value)
        self.final_tolerance = float(self.get_parameter('final_tolerance').value)
        self.path_complete_pub = self.create_publisher(Bool, '/path_complete', 10)
        self.path_points = []
        self.current_index = 0
        self.path_received = False
        self.odom_received = False
        self.path_complete = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.heading_pub = self.create_publisher(Float32, '/cmd_heading', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)
        self.depth_pub = self.create_publisher(Float32, '/cmd_depth', 10)

        self.timer = self.create_timer(0.1, self.follow_path)

        self.get_logger().info('Path follower node started.')

    def path_callback(self, msg: Path):
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        complete_msg = Bool()
        complete_msg.data = False
        self.path_complete_pub.publish(complete_msg)

        self.path_complete = False
        self.path_received = len(self.path_points) > 0

        if not self.path_received:
            self.get_logger().warn('Received empty path.')
            return

        # Default target is the second point, because the first point is home/start
        self.current_index = 1 if len(self.path_points) > 1 else 0

        self.get_logger().info(
            f'Received path with {len(self.path_points)} waypoints. '
            f'Starting target index = {self.current_index}'
        )

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def publish_commands(self, heading, speed, depth):
        h = Float32()
        s = Float32()
        d = Float32()

        h.data = float(heading)
        s.data = float(speed)
        d.data = float(depth)

        self.heading_pub.publish(h)
        self.speed_pub.publish(s)
        self.depth_pub.publish(d)

    def follow_path(self):
        if not self.path_received or not self.odom_received:
            return

        if self.path_complete or self.current_index >= len(self.path_points):
            self.publish_commands(self.yaw, 0.0, self.target_depth)
            return

        target_x, target_y = self.path_points[self.current_index]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)

        # Check if current waypoint reached
        tolerance = self.final_tolerance if self.current_index == len(self.path_points) - 1 else self.waypoint_tolerance
        if distance <= tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_index}: ({target_x:.2f}, {target_y:.2f})')
            self.current_index += 1

            if self.current_index >= len(self.path_points):
                self.path_complete = True
                self.get_logger().info('Path complete.')

                complete_msg = Bool()
                complete_msg.data = True
                self.path_complete_pub.publish(complete_msg)

                self.publish_commands(self.yaw, 0.0, self.target_depth)
                return

            target_x, target_y = self.path_points[self.current_index]
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        if distance > 1.0:
            desired_speed = self.cruise_speed
        elif distance > 0.4:
            desired_speed = 0.12
        else:
            desired_speed = 0.06

        self.get_logger().info(
            f'Target waypoint {self.current_index}: '
            f'({target_x:.2f}, {target_y:.2f}) | '
            f'Current position: ({self.x:.2f}, {self.y:.2f}) | '
            f'Distance: {distance:.2f}'
)

        self.publish_commands(desired_heading, desired_speed, self.target_depth)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()