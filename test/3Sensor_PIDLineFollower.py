#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Three-Sensor Line Follower with PD Control
- Middle sensor for line following (PD controller)
- Side sensors for turn detection and intersection handling
- Inversion-safe motor control
"""

import sys
import select
import tty
import termios
import time

from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent

# ===== THREE-SENSOR PD CONFIGURATION =====
KP = 25        # Proportional gain for middle sensor
KD = 4.0       # Derivative gain for middle sensor
BASE_SPEED = 30   # Nominal base forward speed
MAX_SPEED = 85    # Absolute maximum allowed speed per wheel
MIN_SPEED = 10     # Minimum allowed speed when making sharp turns

# Sensor thresholds
MIDDLE_THRESHOLD = 2    # Threshold for middle sensor (line vs non-line)
SIDE_THRESHOLD = 15      # Threshold for side sensors (detecting intersections)
BLACK_THRESHOLD = 8     # Threshold for detecting black (intersections)

# Turn detection and intersection handling
TURN_DETECTION_COUNT = 3  # Number of consecutive detections needed
STRAIGHT_TURN_SPEED = 25  # Speed during straight-line turns
SHARP_TURN_SPEED = 20     # Speed during sharp turns
TURN_COMPLETE_THRESHOLD = 5  # Readings needed to complete turn

# Control parameters
CORRECTION_FILTER_ALPHA = 0.7
MAX_CORRECTION = 20.0
DEAD_ZONE = 1.5

# Sampling
SAMPLE_INTERVAL = 0.02

# ===== MOTOR ORIENTATION FLAG =====
INVERT_MOTORS = True

# ===== SENSORS & MOTORS =====
cl_L = ColorSensor('in4')  # left sensor - for turn detection
cl_R = ColorSensor('in1')  # right sensor - for turn detection  
cl_M = ColorSensor('in2')  # middle sensor - for line following

try:
    cl_L.mode = 'COL-REFLECT'
    cl_R.mode = 'COL-REFLECT'
    cl_M.mode = 'COL-REFLECT'
except Exception:
    pass

mL = LargeMotor(OUTPUT_D)
mR = LargeMotor(OUTPUT_A)

sound = Sound()
sound.beep()

# ===== HELPER: apply speeds centrally with inversion & clamping =====
def apply_motor_speeds(left_speed, right_speed, invert=INVERT_MOTORS):
    """
    left_speed, right_speed: desired speeds in -100..100 (%).
    invert: if True multiply both by -1 to compensate flipped robot.
    Returns (applied_left, applied_right) after clamping and inversion.
    """
    # Clamp to allowed motor range
    l = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    r = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

    # Apply inversion if requested
    if invert:
        l = -l
        r = -r

    # Send to motors
    mL.on(SpeedPercent(l))
    mR.on(SpeedPercent(r))

    return l, r

# ===== THREE-SENSOR LINE FOLLOWER CLASS =====
class ThreeSensorPDFollower:
    def __init__(self, kp, kd, base_speed, max_speed, min_speed):
        self.kp = float(kp)
        self.kd = float(kd)
        self.base_speed = float(base_speed)
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)

        # PD state for middle sensor
        self.last_error = 0.0
        self.last_time = None
        self.filtered_correction = 0.0

        # Turn detection state
        self.state = "FOLLOWING"  # FOLLOWING, TURN_LEFT, TURN_RIGHT, STRAIGHT_TURN
        self.turn_detection_count = 0
        self.turn_complete_count = 0
        self.last_turn_type = None

    def compute(self, left_value, right_value, middle_value):
        """
        Main control logic:
        - Use middle sensor for PD line following
        - Use side sensors to detect turns and intersections
        Returns: (left_speed, right_speed)
        """
        # State machine for different behaviors
        if self.state == "FOLLOWING":
            return self._following_state(left_value, right_value, middle_value)
        elif self.state == "TURN_LEFT":
            return self._turn_left_state(left_value, right_value, middle_value)
        elif self.state == "TURN_RIGHT":
            return self._turn_right_state(left_value, right_value, middle_value)
        elif self.state == "STRAIGHT_TURN":
            return self._straight_turn_state(left_value, right_value, middle_value)

    def _following_state(self, left_value, right_value, middle_value):
        """
        Normal line following with turn detection
        """
        # Check for turns using side sensors
        turn_detected, turn_type = self._detect_turn(left_value, right_value)
        
        if turn_detected:
            self.state = turn_type
            self.turn_complete_count = 0
            print("TURN DETECTED: {}".format(turn_type))
            sound.beep()
            
            # Start the turn maneuver
            if turn_type == "STRAIGHT_TURN":
                return STRAIGHT_TURN_SPEED, STRAIGHT_TURN_SPEED
            elif turn_type == "TURN_LEFT":
                return SHARP_TURN_SPEED * 0.3, SHARP_TURN_SPEED
            elif turn_type == "TURN_RIGHT":
                return SHARP_TURN_SPEED, SHARP_TURN_SPEED * 0.3
        
        # No turn detected, continue normal line following
        return self._normal_line_following(middle_value)

    def _detect_turn(self, left_value, right_value):
        """
        Detect turns based on side sensor readings
        Returns: (detected, turn_type)
        """
        left_black = left_value < BLACK_THRESHOLD
        right_black = right_value < BLACK_THRESHOLD
        
        # Both sensors on black - intersection (straight turn opportunity)
        if left_black and right_black:
            self.turn_detection_count += 1
            if self.turn_detection_count >= TURN_DETECTION_COUNT:
                self.turn_detection_count = 0
                return True, "STRAIGHT_TURN"
        # Left sensor on black - left turn
        elif left_black and not right_black:
            self.turn_detection_count += 1
            if self.turn_detection_count >= TURN_DETECTION_COUNT:
                self.turn_detection_count = 0
                return True, "TURN_LEFT"
        # Right sensor on black - right turn
        elif not left_black and right_black:
            self.turn_detection_count += 1
            if self.turn_detection_count >= TURN_DETECTION_COUNT:
                self.turn_detection_count = 0
                return True, "TURN_RIGHT"
        else:
            # Reset counter if no consistent detection
            self.turn_detection_count = max(0, self.turn_detection_count - 1)
        
        return False, None

    def _normal_line_following(self, middle_value):
        """
        PD control using only the middle sensor
        Error is based on how far the middle sensor is from the line
        """
        now = time.time()
        if self.last_time is None:
            dt = SAMPLE_INTERVAL
        else:
            dt = now - self.last_time
            if dt <= 0.0:
                dt = SAMPLE_INTERVAL
        self.last_time = now

        # Calculate error: positive when sensor sees darker (needs correction)
        # We assume the line is darker than the background
        error = (MIDDLE_THRESHOLD - middle_value) / 10.0  # Scale error

        # Adaptive cruise speed based on how well we're centered
        if abs(error) < 1.0:  # Well centered
            cruise = min(self.max_speed, self.base_speed + 5)
        elif abs(error) < 2.0:  # Moderately off
            cruise = self.base_speed
        else:  # Far off center
            cruise = max(self.min_speed, self.base_speed - 10)

        # PD terms
        P = self.kp * error
        D = self.kd * ((error - self.last_error) / dt)

        self.last_error = error

        correction = P + D

        # Dead zone to avoid micro-corrections
        if abs(error) < DEAD_ZONE:
            correction = 0.0

        # Clamp correction
        correction = max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        # Exponential smoothing
        self.filtered_correction = (CORRECTION_FILTER_ALPHA * self.filtered_correction +
                                    (1.0 - CORRECTION_FILTER_ALPHA) * correction)

        # Compute wheel speeds
        left_speed = cruise - self.filtered_correction
        right_speed = cruise + self.filtered_correction

        # Ensure we don't exceed physical limits
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)

        return left_speed, right_speed

    def _turn_left_state(self, left_value, right_value, middle_value):
        """
        Execute left turn maneuver
        """
        # Check if turn is complete (middle sensor back on line)
        if middle_value < MIDDLE_THRESHOLD:
            self.turn_complete_count += 1
        else:
            self.turn_complete_count = max(0, self.turn_complete_count - 1)
        
        if self.turn_complete_count >= TURN_COMPLETE_THRESHOLD:
            self.state = "FOLLOWING"
            self.turn_complete_count = 0
            print("Left turn completed")
            return self._normal_line_following(middle_value)
        
        # Continue turning left
        return SHARP_TURN_SPEED * 0.3, SHARP_TURN_SPEED

    def _turn_right_state(self, left_value, right_value, middle_value):
        """
        Execute right turn maneuver
        """
        # Check if turn is complete (middle sensor back on line)
        if middle_value < MIDDLE_THRESHOLD:
            self.turn_complete_count += 1
        else:
            self.turn_complete_count = max(0, self.turn_complete_count - 1)
        
        if self.turn_complete_count >= TURN_COMPLETE_THRESHOLD:
            self.state = "FOLLOWING"
            self.turn_complete_count = 0
            print("Right turn completed")
            return self._normal_line_following(middle_value)
        
        # Continue turning right
        return SHARP_TURN_SPEED, SHARP_TURN_SPEED * 0.3

    def _straight_turn_state(self, left_value, right_value, middle_value):
        """
        Handle intersection - go straight through
        """
        # Continue straight for a fixed time/distance
        # For simplicity, we'll use a counter based approach
        self.turn_complete_count += 1
        
        if self.turn_complete_count >= TURN_COMPLETE_THRESHOLD * 2:
            self.state = "FOLLOWING"
            self.turn_complete_count = 0
            print("Straight turn completed")
            return self._normal_line_following(middle_value)
        
        # Continue going straight
        return STRAIGHT_TURN_SPEED, STRAIGHT_TURN_SPEED

# ===== KEYBOARD & MAIN LOOP =====
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())

    pd_controller = ThreeSensorPDFollower(KP, KD, BASE_SPEED, MAX_SPEED, MIN_SPEED)

    print("=== THREE-SENSOR LINE FOLLOWER ===")
    print("Middle sensor: Line following (PD control)")
    print("Side sensors: Turn detection")
    print("Press 's' start/stop, 'r' reset, 'c' calibrate, 'q' quit")

    running = False
    stop_signal = False

    while not stop_signal:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':
                stop_signal = True
            elif c == 's':
                running = not running
                if running:
                    sound.beep()
                    print("STARTING...")
                else:
                    sound.beep()
                    print("STOPPING...")
                    apply_motor_speeds(0.0, 0.0)
            elif c == 'r':
                pd_controller.last_error = 0.0
                pd_controller.last_time = None
                pd_controller.filtered_correction = 0.0
                pd_controller.state = "FOLLOWING"
                pd_controller.turn_detection_count = 0
                pd_controller.turn_complete_count = 0
                print("Controller reset")
            elif c == 'c':
                print("\n=== SENSOR CALIBRATION ===")
                print("Place robot so MIDDLE sensor is on line and press Enter")
                input()
                middle_on_line = cl_M.value()
                print("Middle on line: {}".format(middle_on_line))

                print("Place robot so MIDDLE sensor is off line and press Enter")
                input()
                middle_off_line = cl_M.value()
                print("Middle off line: {}".format(middle_off_line))

                suggested_middle = (middle_on_line + middle_off_line) / 2.0
                print("Suggested MIDDLE_THRESHOLD: {:.0f}".format(suggested_middle))

                print("\nPlace robot so SIDE sensors see intersection (black) and press Enter")
                input()
                left_black = cl_L.value()
                right_black = cl_R.value()
                print("Left on black: {} Right on black: {}".format(left_black, right_black))

                suggested_side = max(left_black, right_black) + 5
                print("Suggested BLACK_THRESHOLD: {:.0f}".format(suggested_side))

        if running and not stop_signal:
            # Read all three sensors
            try:
                left_value = cl_L.value()
                right_value = cl_R.value()
                middle_value = cl_M.value()
            except Exception:
                left_value = cl_L.reflected_light_intensity
                right_value = cl_R.reflected_light_intensity
                middle_value = cl_M.reflected_light_intensity

            left_speed, right_speed = pd_controller.compute(left_value, right_value, middle_value)

            # Send to motors via central function
            applied_l, applied_r = apply_motor_speeds(left_speed, right_speed)

            # Status print
            error = (MIDDLE_THRESHOLD - middle_value) / 10.0
            print("L:{:3d} R:{:3d} M:{:3d} | Err:{:6.2f} | ML:{:5.1f} MR:{:5.1f} | State: {}".format(
                int(left_value), int(right_value), int(middle_value), error, 
                applied_l, applied_r, pd_controller.state))

        elif not stop_signal:
            apply_motor_speeds(0.0, 0.0)

        time.sleep(SAMPLE_INTERVAL)

    # Exit cleanup
    sound.beep()
    apply_motor_speeds(0.0, 0.0)
    print("Program terminated")

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    apply_motor_speeds(0.0, 0.0)