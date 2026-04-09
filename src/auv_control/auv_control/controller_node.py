#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    """
    Convert quaternion to yaw angle (rotation about Z axis).
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PID:
    def __init__(self, kp: float, ki: float, kd: float, limit: float = 10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral_limit = limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error: float, now: Time) -> float:
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = (now - self.prev_time).nanoseconds * 1e-9

        self.prev_time = now

        # Proportional
        p = self.kp * error

        # Integral
        if dt > 0.0:
            self.integral += error * dt
            self.integral = max(
                min(self.integral, self.integral_limit),
                -self.integral_limit
            )
        i = self.ki * self.integral

        # Derivative
        d = 0.0
        if dt > 0.0:
            d = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        # Total control output
        u = p + i + d
        u = max(min(u, self.limit), -self.limit)

        return u


class AUVController(Node):
    """
    Low-level controller.

    Inputs:
      /cmd_heading   [rad]
      /cmd_speed     [m/s]   (desired forward speed)
      /cmd_depth     [m]     (positive downward)
      /odom

    Outputs:
      /thruster_left
      /thruster_right
      /thruster_vertical
    """

    def __init__(self):
        super().__init__('controller_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('max_thrust', 20.0)
        self.declare_parameter('base_thrust_gain', 5.0)
        self.declare_parameter('depth_positive_down', True)

        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.base_thrust_gain = float(self.get_parameter('base_thrust_gain').value)
        self.depth_positive_down = bool(self.get_parameter('depth_positive_down').value)

        # -----------------------------
        # PID controllers
        # -----------------------------
        self.heading_pid = PID(kp=2.0, ki=0.1, kd=0.3, limit=self.max_thrust)
        self.depth_pid = PID(kp=3.0, ki=0.1, kd=0.5, limit=self.max_thrust)

        # -----------------------------
        # Desired commands
        # -----------------------------
        self.desired_heading = 0.0
        self.desired_speed = 0.0
        self.desired_depth = 0.0

        # -----------------------------
        # Current vehicle state
        # -----------------------------
        self.current_yaw = 0.0
        self.current_depth = 0.0
        self.current_speed = 0.0
        self.odom_received = False

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.heading_sub = self.create_subscription(
            Float32,
            '/cmd_heading',
            self.heading_callback,
            10
        )

        self.speed_sub = self.create_subscription(
            Float32,
            '/cmd_speed',
            self.speed_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Float32,
            '/cmd_depth',
            self.depth_callback,
            10
        )

        # -----------------------------
        # Publishers
        # -----------------------------
        self.left_pub = self.create_publisher(Float32, '/thruster_left', 10)
        self.right_pub = self.create_publisher(Float32, '/thruster_right', 10)
        self.vert_pub = self.create_publisher(Float32, '/thruster_vertical', 10)

        # -----------------------------
        # Control loop timer
        # -----------------------------
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('Controller node started.')

    def odom_callback(self, msg: Odometry):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # In ROS/Gazebo, +z is usually upward.
        # If using "depth positive downward", convert depth = -z.
        z_position = msg.pose.pose.position.z
        if self.depth_positive_down:
            self.current_depth = -z_position
        else:
            self.current_depth = z_position

        self.current_speed = msg.twist.twist.linear.x
        self.odom_received = True

    def heading_callback(self, msg: Float32):
        self.desired_heading = msg.data

    def speed_callback(self, msg: Float32):
        self.desired_speed = msg.data

    def depth_callback(self, msg: Float32):
        self.desired_depth = msg.data

    def angle_wrap(self, angle: float) -> float:
        """
        Wrap angle to [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.odom_received:
            return

        now = self.get_clock().now()

        # -----------------------------
        # Heading control
        # -----------------------------
        heading_error = self.angle_wrap(self.desired_heading - self.current_yaw)
        yaw_cmd = self.heading_pid.update(heading_error, now)

        # -----------------------------
        # Forward thrust
        # Simple proportional mapping from desired speed to thrust
        # -----------------------------
        forward_thrust = self.base_thrust_gain * self.desired_speed
        forward_thrust = max(min(forward_thrust, self.max_thrust), -self.max_thrust)

        # -----------------------------
        # Depth control
        # -----------------------------
        depth_error = self.desired_depth - self.current_depth
        vertical_thrust = self.depth_pid.update(depth_error, now)
        vertical_thrust = max(min(vertical_thrust, self.max_thrust), -self.max_thrust)

        # -----------------------------
        # Differential thrust for heading
        # -----------------------------
        left_cmd = forward_thrust - yaw_cmd
        right_cmd = forward_thrust + yaw_cmd

        left_cmd = max(min(left_cmd, self.max_thrust), -self.max_thrust)
        right_cmd = max(min(right_cmd, self.max_thrust), -self.max_thrust)

        # -----------------------------
        # Publish thruster commands
        # -----------------------------
        left_msg = Float32()
        right_msg = Float32()
        vert_msg = Float32()

        left_msg.data = float(left_cmd)
        right_msg.data = float(right_cmd)
        vert_msg.data = float(vertical_thrust)

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.vert_pub.publish(vert_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AUVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()