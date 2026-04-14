#!/usr/bin/env python3
"""
thruster_test_adafruit_openloop.py
Open-loop thruster test for RoboSub Pre-Qual course.
"""

import time
import math
from adafruit_servokit import ServoKit
from motorMap as maps

# ============================================================
# MOTOR MAP
# ============================================================
CHANNELS = [0, 1, 2, 3, 4, 5]

MOTOR_MAP = {
    "m1": 0,
    "m2": 1,
    "m3": 2,
    "m4": 3,
    "m5": 4,
    "m6": 5,
}

# ============================================================
# POWER SETTINGS
# ============================================================
NEUTRAL_PWR   = 0
DRIVE_FWD_PWR =  0.1
DRIVE_REV_PWR = -0.1
UP_ANGLE      = 80
DOWN_ANGLE    = 115

# ============================================================
# TIME SETTINGS
# ============================================================
ARM_DELAY_S   = 5.0
STEP_S        = 2.0
SPEED_M_PER_S = 0.2    # tune this to match your vehicle's actual speed

# ============================================================
# PRE-QUAL COURSE WAYPOINTS  (x, y) in metres
# ============================================================
START_XY = (1.0, 6.25)
WAYPOINTS = [
    (5.0,  6.25),   # WP1 — gate
    (15.0, 2.0),    # WP2 — bottom sweep
    (20.0, 6.25),   # WP3 — far end
    (15.0, 10.0),   # WP4 — top sweep
    (12.0, 6.25),   # WP5 — mid return
    (3.0,  6.25),   # WP6 — back toward start
]
LABELS = ["Start", "WP1 Gate", "WP2 Bottom", "WP3 Far",
          "WP4 Top", "WP5 Mid", "WP6 Return"]


# ============================================================
# THRUSTER RIG
# ============================================================
class ThrusterRig:
    def __init__(self, motor_map: dict):
        self.kit = ServoKit(channels=16)
        self.kit.frequency = 50  # standard for ESCs

        self.motor_map = motor_map
        self.motors = {}

        for name, ch in self.motor_map.items():
            m = self.kit.servo[ch]
            m.set_pulse_width_range(1000, 2000)
            self.motors[name] = m

        self.stop_all()
        print(f"[OK] Initialized ServoKit. Motors: {self.motor_map}")

    # ----------------------------------------------------------
    # LOW-LEVEL
    # ----------------------------------------------------------
    def set_angle(self, name: str, angle: float):
        if name not in self.motors:
            raise ValueError(f"Unknown motor '{name}'. Valid: {list(self.motors.keys())}")
        self.motors[name].angle = angle

    def stop_all(self):
        for name in self.motors:
            self.motors[name].angle = NEUTRAL_PWR

    def arm_all(self):
        print(f"[INFO] Arming ESCs (neutral for {ARM_DELAY_S:.1f}s)...")
        self.stop_all()
        time.sleep(ARM_DELAY_S)
        print("[OK] Armed.")

    # ----------------------------------------------------------
    # BASIC MOVES
    # ----------------------------------------------------------
    def move_forward(self, duration=2.0):
        print(f"[MOVE] forward  angle={DRIVE_FWD_PWR}  duration={duration:.1f}s")
        for name in self.motors:
            self.set_angle(name, DRIVE_FWD_PWR)
        time.sleep(duration)
        self.stop_all()

    def move_backward(self, duration=2.0):
        print(f"[MOVE] backward  angle={DRIVE_REV_PWR}  duration={duration:.1f}s")
        for name in self.motors:
            self.set_angle(name, DRIVE_REV_PWR)
        time.sleep(duration)
        self.stop_all()

    def move_up(self, duration=2.0):
        vertical = ["m2", "m4"]
        print(f"[MOVE] up  angle={UP_ANGLE}  duration={duration:.1f}s")
        self.stop_all()
        for name in vertical:
            if name in self.motors:
                self.set_angle(name, UP_ANGLE)
        time.sleep(duration)
        self.stop_all()

    def move_down(self, duration=2.0):
        vertical = ["m2", "m4"]
        print(f"[MOVE] down  angle={DOWN_ANGLE}  duration={duration:.1f}s")
        self.stop_all()
        for name in vertical:
            if name in self.motors:
                self.set_angle(name, DOWN_ANGLE)
        time.sleep(duration)
        self.stop_all()

    def test_motor(self, name: str):
        print(f"\n[TEST] {name}")
        self.set_angle(name, NEUTRAL_PWR)
        time.sleep(1.0)

        print(f"  forward ({DRIVE_FWD_PWR})")
        self.set_angle(name, DRIVE_FWD_PWR)
        time.sleep(STEP_S)

        self.set_angle(name, NEUTRAL_PWR)
        time.sleep(1.0)

        print(f"  reverse ({DRIVE_REV_PWR})")
        self.set_angle(name, DRIVE_REV_PWR)
        time.sleep(STEP_S)

        self.set_angle(name, NEUTRAL_PWR)
        time.sleep(1.0)
        print("[DONE] motor test complete.")

    # ----------------------------------------------------------
    # OPEN-LOOP COURSE  (inside the class — fixed)
    # ----------------------------------------------------------
    def run_course(self, speed_m_per_s=SPEED_M_PER_S):
        all_points = [START_XY] + WAYPOINTS

        print("\n" + "="*50)
        print("  OPEN-LOOP PRE-QUAL COURSE")
        print("="*50)
        print(f"  Estimated speed: {speed_m_per_s} m/s")
        print(f"  Waypoints: {len(WAYPOINTS)}\n")

        for i in range(len(all_points) - 1):
            p1 = all_points[i]
            p2 = all_points[i + 1]

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dist = math.sqrt(dx**2 + dy**2)
            heading = math.degrees(math.atan2(dy, dx))
            duration = dist / speed_m_per_s

            print(f"[LEG {i+1}] {LABELS[i]} → {LABELS[i+1]}")
            print(f"  From    : {p1}")
            print(f"  To      : {p2}")
            print(f"  Distance: {dist:.2f} m")
            print(f"  Heading : {heading:.1f}°")
            print(f"  Duration: {duration:.1f} s")

            if dx > 0:
                self.move_forward(duration)
            else:
                self.move_backward(duration)

            print(f"  [LEG {i+1} DONE]\n")

        print("[✓] Pre-qual course complete.\n")


# ============================================================
# MENU
# ============================================================
def print_menu(motor_names):
    print("\n=== Thruster Test Menu ===")
    print("Commands:")
    print("  list               -> show motors and channels")
    print("  test <motor>       -> test one motor (fwd/stop/rev/stop)")
    print("  fwd                -> move forward")
    print("  back               -> move backward")
    print("  up                 -> move up (uses m2, m4)")
    print("  down               -> move down (uses m2, m4)")
    print("  stop               -> stop all motors")
    print("  arm                -> send neutral for a few seconds")
    print("  course             -> run full pre-qual course (open loop)")
    print("  quit               -> exit")
    print(f"Motors: {', '.join(motor_names)}")


# ============================================================
# MAIN
# ============================================================
def main():
    rig = ThrusterRig(MOTOR_MAP)
    rig.arm_all()

    motor_names = list(rig.motors.keys())
    print_menu(motor_names)

    while True:
        try:
            cmd = input("\n> ").strip().lower()
            if not cmd:
                continue

            if cmd in ("quit", "exit"):
                break
            elif cmd == "list":
                print("[MOTORS]")
                for name, ch in rig.motor_map.items():
                    print(f"  {name}: channel {ch}")
            elif cmd.startswith("test "):
                name = cmd.split(" ", 1)[1].strip()
                rig.test_motor(name)
            elif cmd == "fwd":
                rig.move_forward()
            elif cmd == "back":
                rig.move_backward()
            elif cmd == "up":
                rig.move_up()
            elif cmd == "down":
                rig.move_down()
            elif cmd == "stop":
                print("[INFO] stopping all motors")
                rig.stop_all()
            elif cmd == "arm":
                rig.arm_all()
            elif cmd == "course":
                rig.run_course()
            else:
                print("[ERR] Unknown command.")
                print_menu(motor_names)

        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C — stopping motors and exiting.")
            break
        except Exception as e:
            print(f"[ERR] {e}")

    rig.stop_all()
    print("[OK] All motors stopped. Bye.")


if __name__ == "__main__":
    main()