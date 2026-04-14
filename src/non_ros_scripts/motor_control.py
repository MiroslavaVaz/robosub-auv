#!/usr/bin/env python3
"""
Allows for manual control of thrusters, based on a command line interface.

Updated spring 2026
"""

import time
from adafruit_servokit import ServoKit, ContinuousServo
import motor_map as maps
import argparse


# Angle settings (based on old code)
NEUTRAL_PWR = 0
DRIVE_FWD_PWR = 0.1
DRIVE_REV_PWR = -0.1
UP_ANGLE = 80
DOWN_ANGLE = 115

# Time settings
ARM_DELAY_S = 5.0
STEP_S = 2.0  # time spinning (seconds)


class ThrusterRig:
    def __init__(self):
        self.kit = ServoKit(channels=16, reference_clock_speed=25_900_000)

        self._motors: dict[str, ContinuousServo] = {}

        for abrev, name in maps.MOTOR_ABREV.items():
            ch = maps.MOTOR_MAP[name]
            m = self.kit.continuous_servo[ch]  # Use servo instead of continuous_servo
            m.set_pulse_width_range(1000, 2000)
            self._motors[name] = m
            self._motors[abrev] = m
        # Safety: stop everything on init
        self.stop_all()
        print(f"[OK] Initialized ServoKit. Motors: {maps.MOTOR_MAP}")

    def list_motors(self):
        for name in self._motors.keys():
            print(f"{name}")

    def get_motor_names(self):
        return list(self._motors.keys())

    def set_power(self, name: str, power: float):
        if name not in self._motors:
            raise ValueError(
                f"Unknown motor '{name}'. Valid: {list(self._motors.keys())}"
            )
        self._motors[name].throttle = power

    def stop_all(self):
        for name in self._motors:
            self._motors[name].throttle = NEUTRAL_PWR

    def test_motor(self, name: str):
        self.set_power(name, .2)
        input("Press any key to stop")
        self.set_power(name, 0)
        # print(f"\n[TEST] {name}")
        # print("  neutral")
        # time.sleep(1.0)

        # print(f"  forward ({DRIVE_FWD_PWR})")
        # self.set_power(name, DRIVE_FWD_PWR)
        # time.sleep(STEP_S)

        # print("  neutral")
        # self.set_power(name, NEUTRAL_PWR)
        # time.sleep(1.0)

        # print(f"  reverse ({DRIVE_REV_PWR})")
        # self.set_power(name, DRIVE_REV_PWR)
        # time.sleep(STEP_S)

        # print("  neutral")
        # self.set_power(name, NEUTRAL_PWR)
        # time.sleep(1.0)
        # print("[DONE] motor test complete.")

    # Basic “moves”
    def drive_fwd(self, pow=0.25):
        for name, mult in maps.MOTOR_HORZ.items():
            self._motors[name].throttle = pow * mult
    
    def turn_left(self, pow=0.25):
        for name, mult in maps.MOTOR_HORZ.items():
            self._motors[name].throttle = pow * mult

    def move_vert(self, pow=-0.15):
        # If you have dedicated vertical thrusters, set them here.
        for name, mult in maps.MOTOR_VERT.items():
            self._motors[name].throttle = pow * mult


def main():
    rig = ThrusterRig()

    directions = ["fwd", "back", "up", "down"]

    def move_func(args):
        pow = args.power
        options = {
            "fwd": lambda: rig.drive_fwd(pow),
            "back": lambda: rig.drive_fwd(-pow),
            "up": lambda: rig.move_vert(pow),
            "down": lambda: rig.move_vert(-pow),
        }
        options[args.direction]()

    # Run through a test sequence for a motor
    def test_motor(args):
        rig.test_motor(args.motor)

    def kill_sub():
        rig.stop_all()
        print("[OK] All motors stopped. Bye.")
        exit()

    parser = argparse.ArgumentParser()
    subs = parser.add_subparsers(help="Command to run", required=True)

    exit_sub = subs.add_parser("quit", aliases=["exit"], help="Parse a single string")
    exit_sub.set_defaults(func=kill_sub)

    list = subs.add_parser("list", help="List motors")
    list.set_defaults(func=lambda a: rig.list_motors())

    test = subs.add_parser("test", help="Test a motor")
    test.add_argument("motor", choices=rig.get_motor_names())
    test.set_defaults(func=test_motor)

    move = subs.add_parser("move", help="move in a direction")
    move.add_argument("direction", choices=directions)
    move.add_argument("power", type=float, default=0.15)
    move.set_defaults(func=move_func)

    stop = subs.add_parser("stop", help="Stop all motors")
    stop.set_defaults(func=lambda a: rig.stop_all())

    parser.print_help()
    while True:
        try:
            cmd = input("\n> ").strip()
            if not cmd:
                continue
            cmd = cmd.split(" ")
            # cmd.insert(0,"blank")
            args = parser.parse_args(cmd)
            print("test",flush=True)
            print(args)
            args.func(args)

        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C detected. Stopping motors and exiting.")
            kill_sub()

        except Exception as e:
            print(f"[ERR] {e}")


if __name__ == "__main__":
    main()
