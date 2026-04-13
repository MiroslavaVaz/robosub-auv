#!/usr/bin/env python3

import time
from adafruit_servokit import ServoKit, ContinuousServo
from motor_map import *

kit = ServoKit(channels=16, reference_clock_speed=25_900_000)

NEUTRAL = 0
FORWARD = 0.2
REVERSE = -0.2


def arm_motors() -> dict[str, ContinuousServo]:
    motors = {}
    for name, ch in MOTOR_MAP.items():
        motor = kit.continuous_servo[ch]
        motor.set_pulse_width_range(1000, 2000)
        motors[name] = motor

        print(f"Arming {name}")
        motor.throttle = NEUTRAL
        time.sleep(0.07)
    return motors


def twitch_all(motors: dict[str, ContinuousServo]):
    for motor in motors.values():
        motor.throttle = NEUTRAL
        time.sleep(0.1)
        motor.throttle = 0.1
        time.sleep(0.2)
        motor.throttle = NEUTRAL
        time.sleep(0.1)


def manual_motor_control():
    print("Press Q to exit")
    while True:
        dat = input("Input <channel>,<power(-1 to 1)>: ").split(",")
        if dat == "Q":
            return
        dat = (int(dat[0]), float(dat[1]))
        kit.continuous_servo[dat[0]].throttle = dat[1]


def run_all_forward(motors: dict[str, ContinuousServo], power=0.1):
    print("Cycles through each motor, running it forward for 1s")
    for name, motor in motors.items():
        print(f"Running {name}:")
        motor.throttle = power
        time.sleep(1)
        motor.throttle = 0
        time.sleep(0.5)


def drive_fwd(pow=0.25):
    for name, mult in MOTOR_HORZ.items():
        motors[name].throttle = pow * mult

def drive_vert(pow=-0.25):
    for name in MOTOR_VERT:
        motors[name].throttle = pow
def stop_all(): 
    for name in MOTOR_MAP:
        motors[name].throttle = NEUTRAL


if __name__ == "__main__":
    motors = arm_motors()
    print("Driving fwd")
    drive_vert(-.2)
    input("press enter to stop")
    stop_all()
