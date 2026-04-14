#!/usr/bin/env python3

import json
import socket
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ------------------ SETTINGS ------------------
JETSON_IP   = "192.168.0.2"
JETSON_PORT = 5005
LOCK_THRESHOLD = 0.80


# ------------------ TCP HELPERS ------------------
def recv_exactly(sock, n: int) -> bytes:
    """Block until exactly n bytes are received."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Jetson disconnected")
        buf += chunk
    return buf


def recv_result(sock) -> dict:
    """Read a 4-byte length header then parse that many bytes as JSON."""
    raw_len = recv_exactly(sock, 4)
    length  = struct.unpack(">I", raw_len)[0]
    payload = recv_exactly(sock, length)
    return json.loads(payload.decode())


# ------------------ ROS2 NODE ------------------
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.publisher_ = self.create_publisher(String, 'vision_data', 10)
        self.sock = None
        self._connect()

        # Poll for new detections from Jetson at 10Hz
        self.timer = self.create_timer(0.1, self.process_frame)

    def _connect(self):
        """Keep trying to connect to the Jetson until successful."""
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to Jetson at {JETSON_IP}:{JETSON_PORT} ...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((JETSON_IP, JETSON_PORT))
                s.settimeout(None)
                self.sock = s
                self.get_logger().info("Connected to Jetson inference server.")
                return
            except (socket.error, OSError) as e:
                self.get_logger().warn(f"Connection failed: {e}. Retrying in 3s ...")
                import time
                time.sleep(3)

    def process_frame(self):
        try:
            result = recv_result(self.sock)

            pred_class  = result.get("top_class", "none")
            confidence  = result.get("top_confidence", 0.0)

            if confidence >= LOCK_THRESHOLD:
                status = "LOCKED"
            else:
                status = "SEARCHING"

            # Publish in exact same format as original camera_cnn_live.py
            msg = String()
            msg.data = f"status:{status},class:{pred_class},confidence:{confidence:.3f}"
            self.publisher_.publish(msg)

            self.get_logger().info(msg.data)

        except (ConnectionError, OSError, struct.error) as e:
            self.get_logger().warn(f"Lost connection to Jetson: {e}. Reconnecting ...")
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self._connect()

    def destroy_node(self):
        if self.sock:
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()