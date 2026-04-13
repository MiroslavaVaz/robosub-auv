#!/usr/bin/env python3

import csv
import datetime
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        self.depth_data = {}
        self.imu_data = {}
        self.hydrophone_data = {}

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"sensor_data_{timestamp}.csv"
        self.filepath = os.path.join(os.getcwd(), self.filename)

        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp',
            'depth_m', 'pressure_mbar', 'temp_c', 'temp_f',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'hydrophone'
        ])

        self.create_subscription(String, 'depth_data', self.depth_callback, 10)
        self.create_subscription(String, 'imu_data', self.imu_callback, 10)
        self.create_subscription(String, 'hydrophone_data', self.hydro_callback, 10)

        self.timer = self.create_timer(0.5, self.write_row)

        self.get_logger().info(f"Logging sensor data to: {self.filepath}")

    def parse_msg(self, msg_data):
        parsed = {}
        parts = msg_data.split(',')

        for part in parts:
            if ':' in part:
                key, value = part.split(':', 1)
                parsed[key.strip()] = value.strip()

        return parsed

    def depth_callback(self, msg):
        if msg.data != "depth_sensor_read_failed":
            self.depth_data = self.parse_msg(msg.data)

    def imu_callback(self, msg):
        self.imu_data = self.parse_msg(msg.data)

    def hydro_callback(self, msg):
        self.hydrophone_data['hydrophone'] = msg.data

    def write_row(self):
        ts = datetime.datetime.now().isoformat()

        self.writer.writerow([
            ts,
            self.depth_data.get('depth_m', ''),
            self.depth_data.get('pressure_mbar', ''),
            self.depth_data.get('temp_c', ''),
            self.depth_data.get('temp_f', ''),
            self.imu_data.get('accel_x', ''),
            self.imu_data.get('accel_y', ''),
            self.imu_data.get('accel_z', ''),
            self.imu_data.get('gyro_x', ''),
            self.imu_data.get('gyro_y', ''),
            self.imu_data.get('gyro_z', ''),
            self.hydrophone_data.get('hydrophone', '')
        ])

        self.file.flush()

    def destroy_node(self):
        if hasattr(self, 'file') and self.file:
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
