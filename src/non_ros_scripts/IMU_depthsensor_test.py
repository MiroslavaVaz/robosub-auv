#!/usr/bin/env python3
"""
Connects to IMU, Depth sensor over I2C and outputs their data to the console. To check basic functionality of these sensors.

Updated spring 2026
"""



import time
import board
import adafruit_icm20x
import ms5837
import threading
import queue
import datetime

# Sensor setup
print("Initializing I2C...")
i2c = board.I2C()

print("Initializing IMU (ICM20649)...")
imu = adafruit_icm20x.ICM20649(i2c)

print("Initializing Depth Sensor (MS5837-30BA)...")
depth_sensor = ms5837.MS5837_30BA()

if not depth_sensor.init():
    raise RuntimeError("Depth sensor initialization failed")

depth_sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

print("Sensors initialized successfully.\n")
print("Keeping vehicle still for baseline depth calibration...")
time.sleep(2)

# Baseline pressure for zero depth
surface_pressure = None

if depth_sensor.read():
    surface_pressure = depth_sensor.pressure(ms5837.UNITS_mbar)
    print(f"Baseline pressure: {surface_pressure:.2f} mbar\n")
else:
    raise RuntimeError("Could not get baseline pressure from depth sensor")

# Command input + logging
cmd_queue = queue.Queue()
recording = False
log_file = None

def command_listener():
    print("Command mode: type 'record' to start logging, 'stop' to stop logging, 'exit' to quit.")
    while True:
        try:
            cmd = input().strip().lower()
        except EOFError:
            break
        if not cmd:
            continue
        cmd_queue.put(cmd)
        if cmd == "exit":
            break

listener_thread = threading.Thread(target=command_listener, daemon=True)
listener_thread.start()

try:
    while True:
        # handle commands from std input
        while True: #Empty command queue
            try:
                cmd = cmd_queue.get_nowait()
            except queue.Empty:
                break

            if cmd == "record":
                if not recording:
                    filename = datetime.datetime.now().strftime("sensor_data_%Y%m%d_%H%M%S.csv")
                    log_file = open(filename, "w", buffering=1)
                    log_file.write("timestamp,depth_m,pressure_mbar,temp_c,temp_f,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
                    recording = True
                    print(f"Recording started: {filename}")
                else:
                    print("Already recording.")
            elif cmd == "stop":
                if recording and log_file:
                    log_file.close()
                    print("Recording stopped.")
                    log_file = None
                    recording = False
                else:
                    print("Recording is not active.")
            elif cmd == "exit":
                raise KeyboardInterrupt
            else:
                print(f"Unknown command: '{cmd}'. Use record/stop/exit.")

        # ----- Depth sensor -----
        if depth_sensor.read():
            pressure = depth_sensor.pressure(ms5837.UNITS_mbar)
            temperature = depth_sensor.temperature(ms5837.UNITS_Centigrade)
            depth_est = (pressure - surface_pressure) / 100.0
        else:
            pressure = None
            temperature = None
            depth_est = None

        # ----- IMU -----
        accel_x, accel_y, accel_z = imu.acceleration   # m/s^2
        gyro_x, gyro_y, gyro_z = imu.gyro             # rad/s

        # ----- Print results -----
        print("------ SENSOR READINGS ------")

        if depth_est is not None:
            print(f"Depth est:    {depth_est:.3f} m")
            print(f"Pressure:     {pressure:.2f} mbar")
            temp_c = temperature
            temp_f = (temp_c * 9/5) + 32
            print(f"Temperature:  {temp_f:.2f} F ({temp_c:.2f} C)")
        else:
            print("Depth sensor read failed")

        print(f"Accel (m/s^2): X={accel_x:.2f}  Y={accel_y:.2f}  Z={accel_z:.2f}")
        print(f"Gyro (rad/s):  X={gyro_x:.3f}  Y={gyro_y:.3f}  Z={gyro_z:.3f}")
        print()

        if recording and log_file:
            ts = datetime.datetime.now().isoformat()
            temp_f = (temperature * 9/5) + 32 if temperature is not None else None
            log_file.write(
                f"{ts},{depth_est if depth_est is not None else ''},{pressure if pressure is not None else ''},"
                f"{temperature if temperature is not None else ''},{temp_f if temperature is not None else ''},"
                f"{accel_x:.2f},{accel_y:.2f},{accel_z:.2f},{gyro_x:.3f},{gyro_y:.3f},{gyro_z:.3f}\n"
            )

        time.sleep(1)

except KeyboardInterrupt:
    print("\nStop recording.")

finally:
    if log_file:
        log_file.close()
        print("Log file closed.")