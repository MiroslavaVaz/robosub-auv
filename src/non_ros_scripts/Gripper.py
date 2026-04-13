# gripper_test.py - Servo control using Adafruit PWM HAT (PCA9685)
import time
from adafruit_servokit import ServoKit

# PWM HAT settings
KIT_CHANNELS = 16
SERVO_CHANNEL = 0

# Servo angle values (0-180)
OPEN_ANGLE = 30
CLOSE_ANGLE = 150
HOLD_ANGLE = 90

def init_servokit():
    kit = ServoKit(channels=KIT_CHANNELS)
    servo = kit.servo[SERVO_CHANNEL]
    servo.set_pulse_width_range(1100, 1900)
    return servo

servo = init_servokit()

def set_position(angle, hold_time=1.0):
    """Set servo position in degrees (0..180)."""
    angle = max(0, min(180, angle))
    servo.angle = angle
    time.sleep(hold_time)

def gripper_open():
    """Open the gripper."""
    set_position(OPEN_ANGLE, 1.5)

def gripper_close():
    """Close the gripper."""
    set_position(CLOSE_ANGLE, 1.5)

def gripper_hold():
    """Hold at neutral position."""
    set_position(HOLD_ANGLE, 0.5)

if __name__ == '__main__':
    try:
        print('Opening...')
        gripper_open()

        print('Holding mid...')
        gripper_hold()

        print('Closing...')
        gripper_close()

        print('Holding mid...')
        gripper_hold()

        print('Test done.')
    except Exception as e:
        print(f'Error during gripper sequence: {e}')
    finally:
        print('Grip sequence complete.')
