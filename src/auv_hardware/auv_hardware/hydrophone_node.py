#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HydrophoneNode(Node):
    def __init__(self):
        super().__init__('hydrophone_node')
        self.publisher_ = self.create_publisher(String, 'hydrophone_data', 10)
        self.timer = self.create_timer(1.0, self.publish_placeholder)
        self.get_logger().warning("Hydrophone node is a placeholder. Real hydrophone code not added yet.")

    def publish_placeholder(self):
        msg = String()
        msg.data = "hydrophone_placeholder"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HydrophoneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()