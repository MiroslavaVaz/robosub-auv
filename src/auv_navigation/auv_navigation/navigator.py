#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool


class Navigator(Node):
    """
    Takes a Path, follows waypoints one-by-one.
    Outputs desired heading, speed, depth, and path completion flag.
    """

    def __init__(self):
        super().__init__('navigator')

        # Parameters
        self.declare_parameter('waypoint_tolerance', 0.5)   # meters
        self.declare_parameter('cruise_speed', 0.5)         # m/s
        self.declare_parameter('desired_depth', 1.0)        # meters, positive downward

        self.wp_tol = float(self.get_parameter('waypoint_tolerance').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.desired_depth = float(self.get_parameter('desired_depth').value)

        # State
        self.current_path = []
        self.current_index = 0
        self.current_pose = None
        self.path_finished = False

        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Publishers
        self.heading_pub = self.create_publisher(Float32, '/cmd_heading', 10)
        self.speed_pub = self.create_publisher(Float32, '/cmd_speed', 10)
        self.depth_pub = self.create_publisher(Float32, '/cmd_depth', 10)
        self.complete_pub = self.create_publisher(Bool, '/path_complete', 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Navigator node started.')

    def path_callback(self, msg: Path):
        self.current_path = msg.poses
        self.current_index = 0
        self.path_finished = False

        if len(self.current_path) > 0:
            self.get_logger().info(
                f'Navigator received path with {len(self.current_path)} waypoints.'
            )

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def publish_stop(self):
        speed_msg = Float32()
        speed_msg.data = 0.0
        self.speed_pub.publish(speed_msg)

    def publish_depth(self):
        depth_msg = Float32()
        depth_msg.data = self.desired_depth
        self.depth_pub.publish(depth_msg)

    def publish_complete(self, done: bool):
        complete_msg = Bool()
        complete_msg.data = done
        self.complete_pub.publish(complete_msg)

    def control_loop(self):
        if self.current_pose is None:
            return

        # Always hold desired depth
        self.publish_depth()

        if not self.current_path:
            self.publish_stop()
            self.publish_complete(False)
            return

        if self.current_index >= len(self.current_path):
            if not self.path_finished:
                self.get_logger().info('Reached final waypoint.')
                self.path_finished = True
            self.publish_stop()
            self.publish_complete(True)
            return

        target_pose: PoseStamped = self.current_path[self.current_index]

        dx = target_pose.pose.position.x - self.current_pose.position.x
        dy = target_pose.pose.position.y - self.current_pose.position.y
        dist = math.hypot(dx, dy)

        # Move to next waypoint if close enough
        if dist < self.wp_tol:
            self.current_index += 1
            return

        desired_yaw = math.atan2(dy, dx)

        heading_msg = Float32()
        heading_msg.data = desired_yaw
        self.heading_pub.publish(heading_msg)

        speed_msg = Float32()
        speed_msg.data = self.cruise_speed
        self.speed_pub.publish(speed_msg)

        self.publish_complete(False)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()