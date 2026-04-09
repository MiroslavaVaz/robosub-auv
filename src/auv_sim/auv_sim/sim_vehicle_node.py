#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class SimVehicleNode(Node):
    """
    Simple simulated AUV vehicle model.

    Inputs:
      /thruster_left
      /thruster_right
      /thruster_vertical

    Output:
      /odom

    This is a lightweight preliminary simulation for plotting
    planned vs. actual trajectory before real pool testing.
    """

    def __init__(self):
        super().__init__('sim_vehicle_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('forward_gain', 0.020)
        self.declare_parameter('yaw_gain', 0.22)
        self.declare_parameter('vertical_gain', 0.03)
        self.declare_parameter('linear_damping', 0.4)
        self.declare_parameter('yaw_damping', 0.50)
        self.declare_parameter('vertical_damping', 0.10)
        self.declare_parameter('current_drift_x', 0.001)
        self.declare_parameter('current_drift_y', -0.0005)

        self.dt = float(self.get_parameter('dt').value)
        self.forward_gain = float(self.get_parameter('forward_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.vertical_gain = float(self.get_parameter('vertical_gain').value)

        self.linear_damping = float(self.get_parameter('linear_damping').value)
        self.yaw_damping = float(self.get_parameter('yaw_damping').value)
        self.vertical_damping = float(self.get_parameter('vertical_damping').value)

        self.current_drift_x = float(self.get_parameter('current_drift_x').value)
        self.current_drift_y = float(self.get_parameter('current_drift_y').value)

        # -----------------------------
        # Vehicle state
        # -----------------------------
        self.x = -7.8
        self.y = 2.25
        self.z = -1.0   # negative z means underwater
        self.yaw = 0.0

        self.v_forward = 0.0
        self.v_vertical = 0.0
        self.yaw_rate = 0.0

        # Latest thruster commands
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.vertical_cmd = 0.0

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.left_sub = self.create_subscription(
            Float32, '/thruster_left', self.left_callback, 10
        )
        self.right_sub = self.create_subscription(
            Float32, '/thruster_right', self.right_callback, 10
        )
        self.vert_sub = self.create_subscription(
            Float32, '/thruster_vertical', self.vert_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(self.dt, self.update_vehicle)

        self.get_logger().info('Sim vehicle node started.')

    def left_callback(self, msg: Float32):
        self.left_cmd = msg.data

    def right_callback(self, msg: Float32):
        self.right_cmd = msg.data

    def vert_callback(self, msg: Float32):
        self.vertical_cmd = msg.data

    def update_vehicle(self):
        # -----------------------------
        # Convert thrusters to motion
        # -----------------------------
        avg_thrust = 0.5 * (self.left_cmd + self.right_cmd)
        diff_thrust = self.right_cmd - self.left_cmd

        # First-order dynamics with damping
        forward_accel = self.forward_gain * avg_thrust - self.linear_damping * self.v_forward
        yaw_accel = self.yaw_gain * diff_thrust - self.yaw_damping * self.yaw_rate
        vertical_accel = self.vertical_gain * self.vertical_cmd - self.vertical_damping * self.v_vertical

        self.v_forward += forward_accel * self.dt
        self.yaw_rate += yaw_accel * self.dt
        self.v_vertical += vertical_accel * self.dt

        # Update heading
        self.yaw += self.yaw_rate * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Body-forward motion projected into world frame
        vx = self.v_forward * math.cos(self.yaw) + self.current_drift_x
        vy = self.v_forward * math.sin(self.yaw) + self.current_drift_y

        self.x += vx * self.dt
        self.y += vy * self.dt
        self.z += self.v_vertical * self.dt

        # -----------------------------
        # Publish odom
        # -----------------------------
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = quaternion_from_yaw(self.yaw)

        odom.twist.twist.linear.x = self.v_forward
        odom.twist.twist.linear.z = self.v_vertical
        odom.twist.twist.angular.z = self.yaw_rate

        self.odom_pub.publish(odom)

        # -----------------------------
        # Publish TF
        # -----------------------------
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.z
        tf_msg.transform.rotation = quaternion_from_yaw(self.yaw)

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimVehicleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()