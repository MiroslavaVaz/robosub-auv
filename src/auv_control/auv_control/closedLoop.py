#!/usr/bin/env python3
"""
thruster_test_adafruit.py
Closed-loop autonomous navigation for RoboSub Pre-Qual course.
Sensors: IMU (heading), Depth (pressure), Camera (gate/marker)
"""

import time
import math
from adafruit_servokit import ServoKit
from motor_map import *

# ============================================================
# SENSOR IMPORTS — replace stubs with your actual drivers
# ============================================================
try:
    import board
    import adafruit_bno055          # IMU
    import adafruit_ms5837          # Depth / pressure sensor
    IMU_AVAILABLE   = True
    DEPTH_AVAILABLE = True
except ImportError:
    IMU_AVAILABLE   = False
    DEPTH_AVAILABLE = False
    print("[WARN] Sensor libraries not found — running in simulation mode.")

try:
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    print("[WARN] OpenCV not found — camera disabled.")

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
# POWER / ANGLE SETTINGS
# ============================================================
NEUTRAL_PWR    = 0
DRIVE_FWD_PWR  =  0.1
DRIVE_REV_PWR  = -0.1
UP_ANGLE       = 80
DOWN_ANGLE     = 115

VERTICAL_MOTORS = ["m2", "m4"]   # motors used for heave
SURGE_MOTORS    = ["m1", "m3"]   # motors used for forward/back
SWAY_MOTORS     = ["m5", "m6"]   # motors used for left/right strafe

# ============================================================
# NAVIGATION SETTINGS
# ============================================================
SPEED_M_PER_S       = 0.2      # estimated forward speed — tune this
TARGET_DEPTH_M      = 1.0      # depth to hold during course (metres)
DEPTH_TOLERANCE_M   = 0.05     # +/- acceptable depth error
HEADING_TOLERANCE   = 5.0      # degrees — acceptable heading error
HEADING_KP          = 0.002    # P-gain for heading correction
DEPTH_KP            = 0.003    # P-gain for depth correction
LOOP_HZ             = 10       # control loop rate

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

# Time settings
ARM_DELAY_S = 5.0
STEP_S      = 2.0


# ============================================================
# SENSOR INTERFACE  (replace internals with your real drivers)
# ============================================================
class SensorSuite:
    def __init__(self):
        self.imu   = None
        self.depth = None
        self.cam   = None

        if IMU_AVAILABLE:
            try:
                i2c = board.I2C()
                self.imu = adafruit_bno055.BNO055_I2C(i2c)
                print("[OK] IMU connected.")
            except Exception as e:
                print(f"[WARN] IMU init failed: {e}")

        if DEPTH_AVAILABLE:
            try:
                i2c = board.I2C()
                self.depth_sensor = adafruit_ms5837.MS5837_30BA(i2c)
                self.depth_sensor.init()
                print("[OK] Depth sensor connected.")
            except Exception as e:
                print(f"[WARN] Depth sensor init failed: {e}")

        if CAMERA_AVAILABLE:
            try:
                self.cam = cv2.VideoCapture(0)
                print("[OK] Camera connected.")
            except Exception as e:
                print(f"[WARN] Camera init failed: {e}")

    def get_heading_deg(self):
        """Return yaw in degrees (0–360). 0 = +x pool axis."""
        if self.imu:
            try:
                euler = self.imu.euler          # (heading, roll, pitch)
                if euler and euler[0] is not None:
                    return euler[0]
            except Exception:
                pass
        # Simulation stub
        return 0.0

    def get_depth_m(self):
        """Return depth in metres (positive = deeper)."""
        if DEPTH_AVAILABLE and hasattr(self, 'depth_sensor'):
            try:
                if self.depth_sensor.read():
                    return self.depth_sensor.depth()
            except Exception:
                pass
        # Simulation stub
        return TARGET_DEPTH_M

    def camera_sees_gate(self):
        """
        Basic gate detection: look for two vertical dark posts.
        Returns True if gate detected in frame.
        Replace with your actual vision pipeline.
        """
        if not self.cam:
            return False
        ret, frame = self.cam.read()
        if not ret:
            return False
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        tall = [c for c in contours
                if cv2.boundingRect(c)[3] > frame.shape[0] * 0.4]
        return len(tall) >= 2

    def release(self):
        if self.cam:
            self.cam.release()


# ============================================================
# THRUSTER RIG
# ============================================================
class ThrusterRig:
    def __init__(self, motor_map: dict):
        self.kit = ServoKit(channels=16)
        self.kit.frequency = 50

        self.motor_map = motor_map
        self.motors = {}

        for name, ch in self.motor_map.items():
            m = self.kit.servo[ch]
            m.set_pulse_width_range(1000, 2000)
            self.motors[name] = m

        self.sensors = SensorSuite()
        self.stop_all()
        print(f"[OK] Initialized ServoKit. Motors: {self.motor_map}")

    # ----------------------------------------------------------
    # LOW-LEVEL COMMANDS
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
    # BASIC MOVES (manual / open-loop)
    # ----------------------------------------------------------
    def move_forward(self, duration=2.0):
        print(f"[MOVE] forward {duration:.1f}s")
        for name in SURGE_MOTORS:
            if name in self.motors:
                self.set_angle(name, DRIVE_FWD_PWR)
        time.sleep(duration)
        self.stop_all()

    def move_backward(self, duration=2.0):
        print(f"[MOVE] backward {duration:.1f}s")
        for name in SURGE_MOTORS:
            if name in self.motors:
                self.set_angle(name, DRIVE_REV_PWR)
        time.sleep(duration)
        self.stop_all()

    def move_up(self, duration=2.0):
        print(f"[MOVE] up {duration:.1f}s")
        self.stop_all()
        for name in VERTICAL_MOTORS:
            if name in self.motors:
                self.set_angle(name, UP_ANGLE)
        time.sleep(duration)
        self.stop_all()

    def move_down(self, duration=2.0):
        print(f"[MOVE] down {duration:.1f}s")
        self.stop_all()
        for name in VERTICAL_MOTORS:
            if name in self.motors:
                self.set_angle(name, DOWN_ANGLE)
        time.sleep(duration)
        self.stop_all()

    def test_motor(self, name: str):
        print(f"\n[TEST] {name}")
        self.set_angle(name, NEUTRAL_PWR);  time.sleep(1.0)
        print(f"  fwd ({DRIVE_FWD_PWR})")
        self.set_angle(name, DRIVE_FWD_PWR); time.sleep(STEP_S)
        self.set_angle(name, NEUTRAL_PWR);  time.sleep(1.0)
        print(f"  rev ({DRIVE_REV_PWR})")
        self.set_angle(name, DRIVE_REV_PWR); time.sleep(STEP_S)
        self.set_angle(name, NEUTRAL_PWR);  time.sleep(1.0)
        print("[DONE]")

    # ----------------------------------------------------------
    # CLOSED-LOOP PRIMITIVES
    # ----------------------------------------------------------
    def _apply_depth_correction(self):
        """
        Read depth sensor and nudge vertical thrusters to hold TARGET_DEPTH_M.
        Returns the depth error for logging.
        """
        depth = self.sensors.get_depth_m()
        error = depth - TARGET_DEPTH_M          # positive = too deep

        if abs(error) < DEPTH_TOLERANCE_M:
            correction = NEUTRAL_PWR
        else:
            correction = -error * DEPTH_KP      # negative error → go up
            correction = max(-0.05, min(0.05, correction))  # clamp

        for name in VERTICAL_MOTORS:
            if name in self.motors:
                self.set_angle(name, correction)
        return depth, error

    def _apply_heading_correction(self, target_heading_deg):
        """
        Read IMU and apply differential surge correction to maintain heading.
        Returns heading error.
        """
        current = self.sensors.get_heading_deg()
        error = target_heading_deg - current

        # Wrap to -180..+180
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        correction = error * HEADING_KP
        correction = max(-0.05, min(0.05, correction))

        # Differential: port faster → turns starboard, starboard faster → turns port
        for name in SURGE_MOTORS:
            base = DRIVE_FWD_PWR
            if name == SURGE_MOTORS[0]:
                self.set_angle(name, base + correction)
            else:
                self.set_angle(name, base - correction)

        return current, error

    def _heading_to(self, p1, p2):
        """Return the pool-frame heading (deg) from p1 to p2."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.degrees(math.atan2(dy, dx)) % 360

    # ----------------------------------------------------------
    # CLOSED-LOOP LEG: drive from p1 toward p2 with heading+depth hold
    # ----------------------------------------------------------
    def drive_leg_closed_loop(self, p1, p2, label=""):
        dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        target_heading = self._heading_to(p1, p2)
        duration = dist / SPEED_M_PER_S
        dt = 1.0 / LOOP_HZ

        print(f"\n[LEG] {label}")
        print(f"  {p1} → {p2}")
        print(f"  Distance: {dist:.2f}m  Heading: {target_heading:.1f}°  "
              f"Est. time: {duration:.1f}s")

        t_start = time.time()
        while time.time() - t_start < duration:
            heading, h_err = self._apply_heading_correction(target_heading)
            depth,   d_err = self._apply_depth_correction()

            elapsed = time.time() - t_start
            print(f"  t={elapsed:5.1f}s  hdg={heading:6.1f}° (err={h_err:+.1f}°)  "
                  f"depth={depth:.2f}m (err={d_err:+.3f}m)", end="\r")

            time.sleep(dt)

        self.stop_all()
        print(f"\n  [LEG DONE] {label}")

    # Pre Qualification Course
    def run_course(self):
        all_points = [START_XY] + WAYPOINTS

        print("\n" + "="*55)
        print("  AUTONOMOUS PRE-QUAL COURSE — CLOSED LOOP")
        print("="*55)
        print(f"  Target depth : {TARGET_DEPTH_M} m")
        print(f"  Est. speed   : {SPEED_M_PER_S} m/s")
        print(f"  Waypoints    : {len(WAYPOINTS)}")

        # Dive to target depth first
        print("\n[DIVE] Submerging to target depth...")
        self._dive_to_depth(TARGET_DEPTH_M)

        for i in range(len(all_points) - 1):
            p1 = all_points[i]
            p2 = all_points[i + 1]
            label = f"Leg {i+1}: {LABELS[i]} → {LABELS[i+1]}"

            # Camera gate check on approach to WP1
            if i == 0 and self.sensors.cam:
                print("\n[VISION] Watching for gate...")
                gate_seen = self._approach_with_camera(p1, p2, label)
                if gate_seen:
                    print("  [VISION] Gate confirmed — continuing.")
            else:
                self.drive_leg_closed_loop(p1, p2, label)

        self.stop_all()
        print("\n[✓] Pre-qual course complete.\n")

    def _dive_to_depth(self, target_m, timeout=20.0):
        """Descend until depth sensor reads target_m."""
        t_start = time.time()
        while time.time() - t_start < timeout:
            depth = self.sensors.get_depth_m()
            if abs(depth - target_m) < DEPTH_TOLERANCE_M:
                print(f"  [DIVE] At depth {depth:.2f}m ✓")
                self.stop_all()
                return
            for name in VERTICAL_MOTORS:
                if name in self.motors:
                    self.set_angle(name, DOWN_ANGLE)
            time.sleep(0.1)
        self.stop_all()
        print("  [DIVE] Timeout — proceeding at current depth.")

    def _approach_with_camera(self, p1, p2, label):
        """
        Drive toward gate while watching camera.
        Returns True if gate was detected.
        """
        dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        target_heading = self._heading_to(p1, p2)
        duration = dist / SPEED_M_PER_S
        dt = 1.0 / LOOP_HZ
        gate_detected = False

        t_start = time.time()
        while time.time() - t_start < duration:
            self._apply_heading_correction(target_heading)
            self._apply_depth_correction()

            if self.sensors.camera_sees_gate():
                gate_detected = True
                print("\n  [VISION] Gate detected!")

            time.sleep(dt)

        self.stop_all()
        return gate_detected


# MENU
def print_menu(motor_names):
    print("\n=== Thruster Test Menu ===")
    print("  list               -> show motors and channels")
    print("  test <motor>       -> test one motor (fwd/stop/rev/stop)")
    print("  fwd                -> move forward (open loop)")
    print("  back               -> move backward (open loop)")
    print("  up                 -> move up")
    print("  down               -> move down")
    print("  stop               -> stop all motors")
    print("  arm                -> send neutral for a few seconds")
    print("  course             -> run full pre-qual course (closed loop)")
    print("  quit               -> exit")
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

            if cmd in ("quit", "exit"):
                break
            elif cmd == "list":
                for name, ch in rig.motor_map.items():
                    print(f"  {name}: channel {ch}")
            elif cmd.startswith("test "):
                rig.test_motor(cmd.split(" ", 1)[1].strip())
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
            print("\n[INFO] Ctrl+C — stopping.")
            break
        except Exception as e:
            print(f"[ERR] {e}")

    rig.stop_all()
    rig.sensors.release()
    print("[OK] All motors stopped. Bye.")


if __name__ == "__main__":
    main()