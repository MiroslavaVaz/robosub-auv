#!/usr/bin/env python3

import time
import board
import ms5837
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_node')

        self.publisher_ = self.create_publisher(String, 'depth_data', 10)

        self.get_logger().info("Initializing I2C for depth sensor...")
        self.i2c = board.I2C()

        self.get_logger().info("Initializing MS5837 depth sensor...")
        self.depth_sensor = ms5837.MS5837_30BA()

        if not self.depth_sensor.init():
            raise RuntimeError("Depth sensor initialization failed")

        self.depth_sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

        self.get_logger().info("Keeping vehicle still for baseline depth calibration...")
        time.sleep(2)

        if self.depth_sensor.read():
            self.surface_pressure = self.depth_sensor.pressure(ms5837.UNITS_mbar)
            self.get_logger().info(f"Baseline pressure: {self.surface_pressure:.2f} mbar")
        else:
            raise RuntimeError("Could not get baseline pressure from depth sensor")

        self.latest_data = {
            'depth_m': None,
            'pressure_mbar': None,
            'temp_c': None,
            'temp_f': None
        }

        self.timer = self.create_timer(0.5, self.read_depth_sensor)

    def read_depth_sensor(self):
        msg = String()

        if self.depth_sensor.read():
            pressure = self.depth_sensor.pressure(ms5837.UNITS_mbar)
            temp_c = self.depth_sensor.temperature(ms5837.UNITS_Centigrade)
            temp_f = (temp_c * 9 / 5) + 32
            depth_m = (pressure - self.surface_pressure) / 100.0

            self.latest_data = {
                'depth_m': depth_m,
                'pressure_mbar': pressure,
                'temp_c': temp_c,
                'temp_f': temp_f
            }

            msg.data = (
                f"depth_m:{depth_m:.3f},"
                f"pressure_mbar:{pressure:.2f},"
                f"temp_c:{temp_c:.2f},"
                f"temp_f:{temp_f:.2f}"
            )

            self.get_logger().info(
                f"Depth: {depth_m:.3f} m | Pressure: {pressure:.2f} mbar | "
                f"Temp: {temp_f:.2f} F ({temp_c:.2f} C)"
            )
        else:
            msg.data = "depth_sensor_read_failed"
            self.get_logger().warning("Depth sensor read failed")

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()