#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SDU - Embodied AI Project : Speedy Gonzales
-   Jose Angel Huerta Rios
-   Andres De Pool
-   Panagiotis Nikolaidis

Ultrasonic & Claw Control with PID Positioning
"""
"""
Notes:
Closed claw position ~-700 without weight and ~-252 with weight
Open claw position ~-2600
"""

import sys
import select
import tty
import termios
import time

from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_D, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_2, INPUT_3

# =======================
# GLOBAL PARAMETERS
# =======================

# PID controller gains for position control
PID_KP = 1.0      # Proportional gain: Higher = faster response, but too high can overshoot/oscillate
PID_KI = 0.0      # Integral gain: Helps eliminate steady-state error, but too high can cause instability
PID_KD = 0.2      # Derivative gain: Damps the response, helps reduce overshoot

PID_TOLERANCE = 2     # Acceptable error in degrees for PID to consider target reached
PID_MAX_TIME = 3      # Maximum time (seconds) PID will try to reach target before giving up

# SCAN PARAMETERS
SCAN_NUM_POSITIONS = 13   # Number of positions to scan (higher = more resolution, slower scan)
SCAN_ANGLE = 45           # Total angle to scan in degrees (wider = more area, but slower)
SCAN_SETTLE_TIME = 0.2    # Time to wait after moving sensor before taking reading (seconds)

# PEAK DETECTION PARAMETERS
PEAK_THRESHOLD = 7       # Minimum difference (cm) to consider a local minimum a "peak" (object)
GRAB_DISTANCE = 2.5         # Distance (cm) at which the robot will try to grab the object

# ROTATION PARAMETERS
ROTATE_DEGREES_PER_ANGLE = 3  # Conversion factor: scan angle degrees to motor degrees (calibrate for your robot)
ROTATE_TOLERANCE = 3          # Acceptable error in degrees for rotation

# MOVEMENT PARAMETERS
APPROACHING_SPEED = 20        # SpeedPercent for forward movement when approaching object
SPEED = 30                    # Speed for manual movement

# CLAW POSITION PARAMETERS
CLAW_OPEN_POS = -840   # Motor position for open claw
CLAW_CLOSED_POS = 1650  # Motor position for closed claw (without weight)
CLAW_GRAB_POS = 2200    # Motor position for closed claw (with weight)
CLAW_PID_KP = PID_KP    # PID gains for claw position (can be tuned separately if needed)
CLAW_PID_KI = PID_KI
CLAW_PID_KD = PID_KD
CLAW_PID_TOLERANCE = 5  # Acceptable error for claw position

# =======================

# Motors
mL = LargeMotor(OUTPUT_A)
mR = LargeMotor(OUTPUT_D)
claw = MediumMotor(OUTPUT_C)
mB = LargeMotor(OUTPUT_B)  # Scanning motor (not used for movement)

# Ultrasonic Sensor
us = UltrasonicSensor(INPUT_2)
# Gyroscope Sensor
gyro = GyroSensor(INPUT_3)

sound = Sound()

def is_data():
    """Check if there is data available on stdin (non-blocking)."""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def pid_control(target, get_position, set_motor, kp=PID_KP, ki=PID_KI, kd=PID_KD, tolerance=PID_TOLERANCE, max_time=PID_MAX_TIME):
    """
    Simple PID controller for motor position.
    target: desired position
    get_position: function to get current position
    set_motor: function to set motor speed
    kp, ki, kd: PID gains
    tolerance: acceptable error
    max_time: timeout in seconds
    """
    integral = 0
    last_error = 0
    start_time = time.time()
    while True:
        current = get_position()
        error = target - current
        integral += error
        derivative = error - last_error
        output = kp * error + ki * integral + kd * derivative
        output = max(min(output, 100), -100)  # Clamp output
        set_motor(output)
        last_error = error
        if abs(error) < tolerance or (time.time() - start_time) > max_time:
            set_motor(0)
            break
        time.sleep(0.05)

def move_claw_to_position(target_pos, claw_motor=claw, kp=CLAW_PID_KP, ki=CLAW_PID_KI, kd=CLAW_PID_KD, tolerance=CLAW_PID_TOLERANCE):
    """
    Moves the claw to a target position using PID control and holds it there.
    target_pos: desired motor position for the claw
    """
    pid_control(target_pos, lambda: claw_motor.position, lambda spd: claw_motor.on(SpeedPercent(spd)), kp=kp, ki=ki, kd=kd, tolerance=tolerance)
    claw_motor.off()

def display_scan_progress(scan_data):
    """
    Displays scan samples in one line, adapting to the number of samples.
    Format: N.sample/Value : --- (1)[##] --- (2)[##] --- ...
    """
    line = "N.sample/Value :"
    for idx, (_, val) in enumerate(scan_data, 1):
        if val is None:
            val_str = "Err"
        else:
            val_str = "%.1f" % val
        line += " --- (%d)[%s]" % (idx, val_str)
    print(line, end='\r')

def scan_area(num_positions=SCAN_NUM_POSITIONS, scan_angle=SCAN_ANGLE, mL=mL, mR=mR, sensor=us, gyro=gyro):
    """
    Rotates robot clockwise by scan_angle/2, then counterclockwise by scan_angle, scanning at each step,
    and finally returns to the initial orientation.
    Only scans during the counterclockwise sweep.
    Returns a list of (gyro_angle, distance) tuples.
    """
    scan_data = []
    initial_gyro = gyro.angle

    # Step 1: Rotate clockwise by scan_angle/2 to starting scan position
    half_angle = scan_angle / 2
    cw_target_gyro = initial_gyro + half_angle
    def set_rotation_pid(spd):
        mL.on(SpeedPercent(spd))
        mR.on(SpeedPercent(-spd))
    pid_control(cw_target_gyro, lambda: gyro.angle, set_rotation_pid, kp=PID_KP, ki=PID_KI, kd=PID_KD, tolerance=ROTATE_TOLERANCE)
    mL.off()
    mR.off()
    print("Starting scan...")
    # Step 2: Counterclockwise scan by scan_angle, taking samples at each step
    def set_rotation_pid_ccw(spd):
        mL.on(SpeedPercent(-spd))
        mR.on(SpeedPercent(spd))
    angle_step = scan_angle / (num_positions - 1)
    for i in range(num_positions):
        sample_angle = cw_target_gyro - i * angle_step  # Move CCW from rightmost to leftmost
 
        pid_control(sample_angle, lambda: gyro.angle, set_rotation_pid_ccw, kp=PID_KP, ki=PID_KI, kd=PID_KD, tolerance=ROTATE_TOLERANCE)
        mL.off()
        mR.off()
        time.sleep(SCAN_SETTLE_TIME)
        try:
            distance = sensor.distance_centimeters
        except Exception:
            distance = None
        scan_data.append((gyro.angle, distance))
        # Display progress in one line
        line = "Scan N.sample/Value :"
        for idx, (_, val) in enumerate(scan_data, 1):
            val_str = "Err" if val is None else "%.1f" % val
            line += " --- (%d)[%s]" % (idx, val_str)
        print(line, end='\r')
    print("Returning to initial position")  # Move to next line after scan

    # Step 3: Return to initial orientation
    pid_control(initial_gyro, lambda: gyro.angle, set_rotation_pid_ccw, kp=PID_KP, ki=PID_KI, kd=PID_KD, tolerance=ROTATE_TOLERANCE)
    mL.off()
    mR.off()

    return scan_data

def rotate_to_peak(scan_angle, peak_angle, mL, mR, kp=PID_KP, tolerance=ROTATE_TOLERANCE):
    """
    Rotates the robot in place using mL and mR to face the detected peak.
    scan_angle: total scan angle (degrees)
    peak_angle: angle (degrees) to rotate to (relative to scan start)
    kp: PID proportional gain for rotation
    tolerance: acceptable error in degrees
    """
    # Assume robot starts facing 0 deg (scan start)
    # Positive angle: turn right, Negative: turn left
    # For simplicity, map angle to motor degrees (may need calibration)
    rotation_degrees = (peak_angle - scan_angle/2) * ROTATE_DEGREES_PER_ANGLE

    # Set target positions for both motors (opposite directions for in-place turn)
    target_L = mL.position + rotation_degrees
    target_R = mR.position - rotation_degrees

    pid_control(target_L, lambda: mL.position, lambda spd: mL.on(SpeedPercent(spd)), kp=kp, tolerance=tolerance)
    pid_control(target_R, lambda: mR.position, lambda spd: mR.on(SpeedPercent(-spd)), kp=kp, tolerance=tolerance)
    mL.off()
    mR.off()

def find_and_grab_peak(scan_data, mL, mR, mB, claw_motor, sensor, sound, peak_threshold=PEAK_THRESHOLD, grab_distance=GRAB_DISTANCE):
    """
    Analyzes scan data to find a peak, rotates robot to face it using gyro, moves forward using all three motors, and grabs.
    scan_data: list of (gyro_angle, distance) tuples
    """
    # Find peaks: a peak is where distance decreases then increases (local minimum)
    peaks = []
    for i in range(1, len(scan_data)-1):
        prev_dist = scan_data[i-1][1]
        curr_dist = scan_data[i][1]
        next_dist = scan_data[i+1][1]
        if prev_dist is not None and curr_dist is not None and next_dist is not None:
            if prev_dist > curr_dist < next_dist and (prev_dist - curr_dist > peak_threshold) and (next_dist - curr_dist > peak_threshold):
                peaks.append((i, scan_data[i]))

    if not peaks:
        print("No peaks detected, likely a wall or no object.")
        return False

    # Move towards the closest peak
    peak_idx, peak = min(peaks, key=lambda x: x[1][1])
    print("\nApproaching peak at scan index %d, angle %.1f, distance %.1f cm" % (peak_idx, peak[0], peak[1]))

    # Rotate robot to face the peak using gyro
    target_gyro = peak[0]
    def set_rotation_pid(spd):
        mL.on(SpeedPercent(spd))
        mR.on(SpeedPercent(-spd))
    pid_control(target_gyro, lambda: gyro.angle, set_rotation_pid, kp=PID_KP, ki=PID_KI, kd=PID_KD, tolerance=ROTATE_TOLERANCE)
    mL.off()
    mR.off()

    # Move forward using all three motors until close enough
    while True:
        try:
            current_distance = sensor.distance_centimeters
        except Exception:
            current_distance = None
        if current_distance is not None and current_distance <= grab_distance:
            print("Object within grabbing distance (%.1f cm). Closing claw." % current_distance)
            move_claw_to_position(CLAW_CLOSED_POS, claw_motor)
            sound.beep()
            mL.off()
            mR.off()
            mB.off()
            break
        mL.on(SpeedPercent(-APPROACHING_SPEED))
        mR.on(SpeedPercent(-APPROACHING_SPEED))
        mB.on(SpeedPercent(APPROACHING_SPEED))
        time.sleep(0.2)
        mL.off()
        mR.off()
        mB.off()
    return True

def show_gyro_data():
    """
    Continuously displays gyroscope angle and rate until any key is pressed.
    """
    print("Press any key to stop displaying gyroscope data.")
    while True:
        try:
            angle = gyro.angle
            rate = gyro.rate
            print("Gyro angle: %d deg | Gyro rate: %d deg/s   " % (angle, rate), end='\r')
        except Exception:
            print("Gyro sensor error.                      ", end='\r')
        time.sleep(0.1)
        if is_data():
            sys.stdin.read(1)
            print()  # Move to next line after stopping
            break

def show_gyro_angle():
    """
    Continuously displays only the gyroscope angle until any key is pressed.
    """
    print("GYRO-ANG mode. Press any key to stop displaying gyroscope angle.")
    while True:
        try:
            angle = gyro.angle
            print("GYRO-ANG : %d deg   " % angle, end='\r')
        except Exception:
            print("Gyro sensor error.   ", end='\r')
        time.sleep(0.1)
        if is_data():
            sys.stdin.read(1)
            print()  # Move to next line after stopping
            break

# Main program
old_settings = termios.tcgetattr(sys.stdin)
sound.beep()

try:
    tty.setcbreak(sys.stdin.fileno())

    print("=== ULTRASONIC & CLAW TEST MENU ===")
    print("Press 'o' to open claw, 'c' to close claw, 'm' move forward (hold to move), 's' start/stop data, 'g' show gyroscope, 'a' GYRO-ANG mode, 'b' tomato search, 'q' quit")

    running = True
    show_data = False  # Flag to control data display

    last_us_val = None  # Store last significant ultrasonic value

    while running:
        # Handle serial input
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':
                running = False
                sound.beep()
            elif c == 'o':
                print("Press and hold 'o' to open claw. Release to stop.")
                claw.on(-SPEED)
                # while True:
                #     if not is_data():
                #         continue
                #     c = sys.stdin.read(1)
                #     if c != 'c':
                #         claw.off()
                #         print("Stopped closing claw. Claw position: %s" % claw.position)
                #         break
            elif c == 'c':
                print("Press and hold 'c' to close claw. Release to stop.")
                claw.on(SPEED)
                # while True:
                #     if not is_data():
                #         continue
                #     c = sys.stdin.read(1)
                #     if c != 'c':
                #         claw.off()
                #         print("Stopped closing claw. Claw position: %s" % claw.position)
                #         break
            elif c == 'm':
                print("Press and hold 'm' to move forward. Release to stop.")
                mL.on(SpeedPercent(-SPEED))
                mR.on(SpeedPercent(-SPEED))
                mB.on(SpeedPercent(SPEED))
                while True:
                    if not is_data():
                        continue
                    c = sys.stdin.read(1)
                    if c != 'm':
                        mL.off()
                        mR.off()
                        mB.off()
                        print("Stopped moving. Left motor position: %s, Right motor position: %s" % (mL.position, mR.position))
                        break
            elif c == 's':
                show_data = not show_data
                if show_data:
                    print("Data display started")
                else:
                    print("Data display stopped")
            elif c == 'g':
                show_gyro_data()
            elif c == 'a':
                show_gyro_angle()
            elif c == 'b':
                print("Starting tomato can search with PID control...")
                scan_data = scan_area()
                found = find_and_grab_peak(scan_data, mL, mR, mB, claw, us, sound)
                if found:
                    print("Tomato can grabbed!")
                else:
                    print("No tomato can found.")

        # Display data if enabled
        if show_data:
            try:
                us_val = us.distance_centimeters
            except Exception:
                us_val = "Ultrasonic sensor error"

            try:
                claw_pos = claw.position
            except Exception:
                claw_pos = "Claw motor error"

            if isinstance(us_val, (int, float)):
                if last_us_val is None or abs(us_val - last_us_val) > 0.2:
                    print("Ultrasonic: [%s] cm, Claw position: [%s]" % (us_val, claw_pos))
                    last_us_val = us_val
            elif us_val == "Ultrasonic sensor error":
                print("Ultrasonic sensor error, Claw position: [%s]" % claw_pos)

        time.sleep(0.05)

    print("Program terminated")

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    mL.off()
    mR.off()
    claw.off()
    mB.off()