#!/usr/bin/env python3
"""
thruster_test_adafruit.py
"""

import time
from adafruit_servokit import ServoKit
from motor_map import *
CHANNELS = [0, 1, 2, 3, 4, 5]

# Map your physical thrusters to PCA9685 channels.
MOTOR_MAP = {
    "m1": 0,
    "m2": 1,
    "m3": 2,
    "m4": 3,
    "m5": 4,
    "m6": 5,
}

# Angle settings (based on old code)
NEUTRAL_PWR = 0
DRIVE_FWD_PWR = .1
DRIVE_REV_PWR = -.1
UP_ANGLE = 80
DOWN_ANGLE = 115

# Time settings
ARM_DELAY_S = 5.0
STEP_S = 2.0 #time spinning (seconds)

class ThrusterRig:
    def __init__(self, motor_map: dict):
        self.kit = ServoKit(channels=16)
        self.kit.frequency = 50  # Standard for servos/ESCs

        self.motor_map = motor_map
        self.motors = {}

        for name, ch in self.motor_map.items():
            m=self.kit.servo[ch]  # Use servo instead of continuous_servo
            m.set_pulse_width_range(1000, 2000)  # 1000-2000 as in old code
            self.motors[name] = m

        # Safety: stop everything on init
        self.stop_all()
        print(f"[OK] Initialized ServoKit. Motors: {self.motor_map}")

    def set_angle(self, name: str, angle: float):
        if name not in self.motors:
            raise ValueError(f"Unknown motor '{name}'. Valid: {list(self.motors.keys())}")
        self.motors[name].angle = angle

    def stop_all(self):
        for name in self.motors:
            self.motors[name].angle = NEUTRAL_PWR

    def arm_all(self):
        # ESCs neutral for a moment after power-up
        print(f"[INFO] Arming ESCs (neutral for {ARM_DELAY_S:.1f}s)...")
        self.stop_all()
        time.sleep(ARM_DELAY_S)
        print("[OK] Armed.")

    def test_motor(self, name: str):
        print(f"\n[TEST] {name}")
        print("  neutral")
        self.set_angle(name, NEUTRAL_PWR)
        time.sleep(1.0)

        print(f"  forward ({DRIVE_FWD_PWR})")
        self.set_angle(name, DRIVE_FWD_PWR)
        time.sleep(STEP_S)

        print("  neutral")
        self.set_angle(name, NEUTRAL_PWR)
        time.sleep(1.0)

        print(f"  reverse ({DRIVE_REV_PWR})")
        self.set_angle(name, DRIVE_REV_PWR)
        time.sleep(STEP_S)

        print("  neutral")
        self.set_angle(name, NEUTRAL_PWR)
        time.sleep(1.0)

        print("[DONE] motor test complete.")

    # Basic “moves” 
    def move_forward(self, duration=2.0):
        # Default: all motors same direction.
        print(f"[MOVE] forward angle={DRIVE_FWD_PWR} duration={duration}s")
        for name in self.motors:
            self.set_angle(name, DRIVE_FWD_PWR)
        time.sleep(duration)
        self.stop_all()

    def move_backward(self, duration=2.0):
        print(f"[MOVE] backward angle={DRIVE_REV_PWR} duration={duration}s")
        for name in self.motors:
            self.set_angle(name, DRIVE_REV_PWR)
        time.sleep(duration)
        self.stop_all()

    def move_up(self, duration=2.0):
        # If you have dedicated vertical thrusters, set them here.
        vertical = ["m2", "m4"]
        print(f"[MOVE] up angle={UP_ANGLE} duration={duration}s (using {vertical})")
        self.stop_all()
        for name in vertical:
            if name in self.motors:
                self.set_angle(name, UP_ANGLE)
        time.sleep(duration)
        self.stop_all()

    def move_down(self, duration=2.0):
        vertical = ["m2", "m4"]
        print(f"[MOVE] down angle={DOWN_ANGLE} duration={duration}s (using {vertical})")
        self.stop_all()
        for name in vertical:
            if name in self.motors:
                self.set_angle(name, DOWN_ANGLE)
        time.sleep(duration)
        self.stop_all()

def print_menu(motor_names):
    print("\n=== Thruster Test Menu ===")
    print("Commands:")
    print("  list                   -> show motors and channels")
    print("  test <motor>           -> test one motor (fwd/stop/rev/stop)")
    print("  fwd                    -> move forward")
    print("  back                   -> move backward")
    print("  up                     -> move up (uses m2,m4 by default)")
    print("  down                   -> move down (uses m2,m4 by default)")
    print("  stop                   -> stop all motors")
    print("  arm                    -> send neutral for a few seconds")
    print("  quit                   -> exit")
    print(f"Motors: {', '.join(motor_names)}")


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
            
            if cmd == "quit" or cmd == "exit":
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

            else:
                print("[ERR] Unknown command.")
                print_menu(motor_names)

        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C detected. Stopping motors and exiting.")
            break
        except Exception as e:
            print(f"[ERR] {e}")

    rig.stop_all()
    print("[OK] All motors stopped. Bye.")

if __name__ == "__main__":
    main()