#!/usr/bin/env python3
"""
vision_mission_node.py — Run this on the RASPBERRY PI.

Bench test mode:
    - Waits for a LOCKED detection on the target class
    - Prints "firetruck detected — press Y to confirm"
    - Only activates thrusters after Y is pressed
    - Safe to run in the lab with AUV on the bench

ROS topics subscribed:
    /vision_data  (std_msgs/String) — from camera_cnn_live

ROS topics published:
    /cmd_heading  (std_msgs/Float32) — desired heading in radians
    /cmd_speed    (std_msgs/Float32) — desired forward speed m/s
    /cmd_depth    (std_msgs/Float32) — desired depth in meters
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

# ------------------ MISSION CONFIG ------------------
TARGET_CLASS    = "firetruck"   # must match your model's class name exactly
DIVE_DEPTH      = 1.0           # meters (not used on bench, but sent to controller)
FORWARD_SPEED   = 0.3           # m/s when confirmed
GATE_PASS_TIME  = 3.0           # seconds to drive after confirmation
BENCH_TEST_MODE = True          # set False for actual pool run


# ------------------ STATES ------------------
IDLE        = "IDLE"
SEARCHING   = "SEARCHING"
AWAITING    = "AWAITING_CONFIRMATION"
CONFIRMED   = "CONFIRMED"
RUNNING     = "RUNNING"
COMPLETE    = "COMPLETE"


class VisionMissionNode(Node):
    def __init__(self):
        super().__init__('vision_mission_node')

        self.state            = IDLE
        self.run_start_time   = None
        self.awaiting_input   = False
        self.user_confirmed   = False
        self.last_class       = ""
        self.last_confidence  = 0.0

        # Subscribers
        self.vision_sub = self.create_subscription(
            String,
            '/vision_data',
            self.vision_callback,
            10
        )

        # Publishers → controller_node
        self.heading_pub = self.create_publisher(Float32, '/cmd_heading', 10)
        self.speed_pub   = self.create_publisher(Float32, '/cmd_speed',   10)
        self.depth_pub   = self.create_publisher(Float32, '/cmd_depth',   10)

        # Control loop at 10Hz
        self.timer = self.create_timer(0.1, self.mission_loop)

        self.get_logger().info('='*50)
        self.get_logger().info('Vision mission node started.')
        self.get_logger().info(f'Target class : {TARGET_CLASS}')
        self.get_logger().info(f'Bench test   : {BENCH_TEST_MODE}')
        self.get_logger().info('='*50)

        self.transition(SEARCHING)

    # ------------------ STATE TRANSITIONS ------------------
    def transition(self, new_state: str):
        self.get_logger().info(f'[STATE] {self.state} → {new_state}')
        self.state = new_state

    # ------------------ VISION CALLBACK ------------------
    def vision_callback(self, msg: String):
        try:
            parts = {}
            for part in msg.data.split(","):
                k, v = part.split(":")
                parts[k.strip()] = v.strip()

            status     = parts.get("status", "")
            det_class  = parts.get("class", "")
            confidence = float(parts.get("confidence", 0.0))

        except Exception as e:
            self.get_logger().warn(f'Failed to parse vision_data: {e}')
            return

        self.last_class      = det_class
        self.last_confidence = confidence

        # Only respond when SEARCHING
        if self.state == SEARCHING:
            if status == "LOCKED" and det_class == TARGET_CLASS:
                self.transition(AWAITING)
                self._prompt_user()

        # If we lose lock while awaiting, go back to searching
        elif self.state == AWAITING:
            if status != "LOCKED" or det_class != TARGET_CLASS:
                self.get_logger().warn('[!] Lost lock before confirmation — back to SEARCHING')
                self.awaiting_input = False
                self.transition(SEARCHING)

    # ------------------ USER PROMPT ------------------
    def _prompt_user(self):
        """Run input prompt in a separate thread so ROS keeps spinning."""
        if self.awaiting_input:
            return
        self.awaiting_input = True

        def ask():
            print("\n" + "="*50)
            print(f"  {TARGET_CLASS.upper()} DETECTED!")
            print(f"  Confidence: {self.last_confidence:.2f}")
            print("="*50)
            print("  Press Y then ENTER to activate thrusters")
            print("  Press N then ENTER to cancel and keep searching")
            print("="*50)

            while True:
                try:
                    response = input("  > ").strip().lower()
                except EOFError:
                    response = "n"

                if response == "y":
                    self.user_confirmed = True
                    self.awaiting_input = False
                    print(f"\n  [OK] Confirmed! Activating thrusters...\n")
                    break
                elif response == "n":
                    self.user_confirmed = False
                    self.awaiting_input = False
                    print(f"\n  [CANCELLED] Going back to searching...\n")
                    self.transition(SEARCHING)
                    break
                else:
                    print("  Please enter Y or N")

        thread = threading.Thread(target=ask, daemon=True)
        thread.start()

    # ------------------ MISSION LOOP ------------------
    def mission_loop(self):

        if self.state == IDLE:
            self.publish_commands(speed=0.0, heading=0.0, depth=0.0)

        elif self.state == SEARCHING:
            # Hold depth, no forward motion
            self.publish_commands(speed=0.0, heading=0.0, depth=DIVE_DEPTH)

        elif self.state == AWAITING:
            # Hold position while waiting for user confirmation
            self.publish_commands(speed=0.0, heading=0.0, depth=DIVE_DEPTH)

            # Check if user confirmed in the input thread
            if self.user_confirmed:
                self.user_confirmed   = False
                self.run_start_time   = self.get_clock().now()
                self.transition(RUNNING)

        elif self.state == RUNNING:
            if BENCH_TEST_MODE:
                # In bench test mode — don't actually run thrusters
                # just log and complete immediately
                self.get_logger().info('[BENCH] Thruster command would fire here.')
                self.get_logger().info('[BENCH] Skipping thruster activation.')
                self.transition(COMPLETE)
            else:
                # Real pool run — drive forward
                self.publish_commands(
                    speed=FORWARD_SPEED,
                    heading=0.0,
                    depth=DIVE_DEPTH
                )
                if self.run_start_time is not None:
                    elapsed = (
                        self.get_clock().now() - self.run_start_time
                    ).nanoseconds * 1e-9
                    if elapsed >= GATE_PASS_TIME:
                        self.transition(COMPLETE)

        elif self.state == COMPLETE:
            self.publish_commands(speed=0.0, heading=0.0, depth=DIVE_DEPTH)
            self.get_logger().info('[DONE] Task complete. Holding position.')

    # ------------------ HELPERS ------------------
    def publish_commands(self, speed: float, heading: float, depth: float):
        speed_msg         = Float32()
        speed_msg.data    = float(speed)
        heading_msg       = Float32()
        heading_msg.data  = float(heading)
        depth_msg         = Float32()
        depth_msg.data    = float(depth)

        self.speed_pub.publish(speed_msg)
        self.heading_pub.publish(heading_msg)
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()