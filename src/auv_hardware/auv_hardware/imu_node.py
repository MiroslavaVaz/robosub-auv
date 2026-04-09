#!/usr/bin/env python3

import board
import adafruit_icm20x
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.publisher_ = self.create_publisher(String, 'imu_data', 10)

        self.get_logger().info("Initializing I2C for IMU...")
        self.i2c = board.I2C()

        self.get_logger().info("Initializing ICM20649 IMU...")
        self.imu = adafruit_icm20x.ICM20649(self.i2c)

        self.latest_data = {
            'accel_x': None,
            'accel_y': None,
            'accel_z': None,
            'gyro_x': None,
            'gyro_y': None,
            'gyro_z': None
        }

        self.timer = self.create_timer(0.5, self.read_imu)

    def read_imu(self):
        accel_x, accel_y, accel_z = self.imu.acceleration
        gyro_x, gyro_y, gyro_z = self.imu.gyro

        self.latest_data = {
            'accel_x': accel_x,
            'accel_y': accel_y,
            'accel_z': accel_z,
            'gyro_x': gyro_x,
            'gyro_y': gyro_y,
            'gyro_z': gyro_z
        }

        msg = String()
        msg.data = (
            f"accel_x:{accel_x:.2f},accel_y:{accel_y:.2f},accel_z:{accel_z:.2f},"
            f"gyro_x:{gyro_x:.3f},gyro_y:{gyro_y:.3f},gyro_z:{gyro_z:.3f}"
        )

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Accel (m/s^2): X={accel_x:.2f} Y={accel_y:.2f} Z={accel_z:.2f} | "
            f"Gyro (rad/s): X={gyro_x:.3f} Y={gyro_y:.3f} Z={gyro_z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()