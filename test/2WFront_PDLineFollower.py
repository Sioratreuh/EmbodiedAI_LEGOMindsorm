#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Center-Line PD Follower (inversion-safe) with Discontinuous Line Support
- Enhanced recovery system with timer reset on line detection
- Simple buzzer tones for program states
- Maintains last correction during line loss timeout
"""

import sys
import select
import tty
import termios
import time

from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent

# ===== CENTER-LINE PD CONFIGURATION =====
KP = 18        # Proportional gain #18
KD = 2.0       # Derivative gain # 2.0
BASE_SPEED = 35   # Nominal base forward speed / 30
MAX_SPEED = 90    # Absolute maximum allowed speed per wheel / 85
MIN_SPEED = 6    # Minimum allowed speed when near line /8

# Thresholds for line detection
LINE_THRESHOLD = 12 #12
LINE_LOSS_TIMEOUT = 0.7  # Time to maintain last correction when line is lost (seconds)
SEARCH_SPEED = 30

# Control parameters
CORRECTION_FILTER_ALPHA = 0.65
MAX_CORRECTION = 18.0
DEAD_ZONE = 1.0

# Sampling error control
SAMPLE_INTERVAL = 0.02

# ===== MOTOR ORIENTATION FLAG =====
INVERT_MOTORS = True

# ===== SENSORS & MOTORS =====
cl_L = ColorSensor('in4')  # left sensor
cl_R = ColorSensor('in1')  # right sensor
try:
    cl_L.mode = 'COL-REFLECT'
    cl_R.mode = 'COL-REFLECT'
except Exception:
    pass

mL = LargeMotor(OUTPUT_D)
mR = LargeMotor(OUTPUT_A)

sound = Sound()
# Simple startup tone
sound.beep()

# ===== HELPER: apply speeds centrally with inversion & clamping =====
def apply_motor_speeds(left_speed, right_speed, invert=INVERT_MOTORS):
    """
    Applies motor speeds with inversion and safety clamping
    """
    l = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    r = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

    if invert:
        l = -l
        r = -r

    mL.on(SpeedPercent(l))
    mR.on(SpeedPercent(r))

    return l, r

# ===== ENHANCED CENTER-LINE FOLLOWER WITH TIMER RESET =====
class CenterLinePDFollower:
    def __init__(self, kp, kd, base_speed, max_speed, min_speed):
        self.kp = float(kp)
        self.kd = float(kd)
        self.base_speed = float(base_speed)
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)

        # PD state
        self.last_error = 0.0
        self.last_time = None
        self.filtered_correction = 0.0

        # Enhanced recovery state with timer reset capability
        self.recovery_state = "NORMAL"  # NORMAL, TIMER_STRAIGHT, SEARCH_360
        self.recovery_start_time = 0.0
        self.last_known_correction = 0.0  # Stores last working correction
        self.consecutive_black_detections = 0  # Counts consecutive line detections
        self.last_turn_direction = "RIGHT"

    def compute(self, left_value, right_value):
        """
        Enhanced state machine with timer reset on line detection
        """
        # Check if line is detected (at least one sensor sees black)
        line_detected = self._is_line_detected(left_value, right_value)
        
        # Check if line is completely lost
        line_lost = self._is_line_lost(left_value, right_value)

        if line_lost:
            current_time = time.time()
            
            if self.recovery_state == "NORMAL":
                # Transition to timer-based movement with last known correction
                self.recovery_state = "TIMER_STRAIGHT"
                self.recovery_start_time = current_time
                # Store last known good correction
                self.last_known_correction = self.filtered_correction
                self.consecutive_black_detections = 0
                print("LINE LOST -> Using last correction for {:.1f}s".format(LINE_LOSS_TIMEOUT))
                return self._timer_straight_recovery()
                
            elif self.recovery_state == "TIMER_STRAIGHT":
                # Check if we should reset the timer due to line detection
                if line_detected:
                    self.consecutive_black_detections += 1
                    # Reset timer if we get consistent line detection
                    if self.consecutive_black_detections >= 2:
                        self.recovery_start_time = current_time
                        self.consecutive_black_detections = 0
                        print("Line detected - resetting timer")
                else:
                    self.consecutive_black_detections = 0

                # Continue using last correction if within timeout
                if current_time - self.recovery_start_time < LINE_LOSS_TIMEOUT:
                    return self._timer_straight_recovery()
                else:
                    # Timer expired, start searching
                    self.recovery_state = "SEARCH_360"
                    print("Timer expired -> starting 360 search")
                    return self._start_360_search()
                    
            elif self.recovery_state == "SEARCH_360":
                return self._continue_360_search()
                
        else:
            # Line detected - reset recovery state
            if self.recovery_state != "NORMAL":
                recovery_duration = time.time() - self.recovery_start_time
                print("Line reacquired after {:.2f}s -> resuming normal PD".format(recovery_duration))
                
            self.recovery_state = "NORMAL"
            self.consecutive_black_detections = 0
            return self._normal_center_line_following(left_value, right_value)

    def _is_line_detected(self, left_value, right_value):
        """Detects when at least one sensor sees the black line"""
        return (left_value <= LINE_THRESHOLD) or (right_value <= LINE_THRESHOLD)

    def _is_line_lost(self, left_value, right_value):
        """Detects when both sensors lose the line"""
        return (left_value > LINE_THRESHOLD) and (right_value > LINE_THRESHOLD)

    def _timer_straight_recovery(self):
        """
        Uses last known correction to maintain trajectory during line loss
        """
        # Apply last known correction to maintain trajectory
        left_speed = self.base_speed - self.last_known_correction
        right_speed = self.base_speed + self.last_known_correction
        
        # Clamp speeds to safe limits
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        
        return left_speed, right_speed

    def _normal_center_line_following(self, left_value, right_value):
        """Standard PD control logic when line is detected"""
        now = time.time()
        if self.last_time is None:
            dt = SAMPLE_INTERVAL
        else:
            dt = now - self.last_time
            if dt <= 0.0:
                dt = SAMPLE_INTERVAL
        self.last_time = now

        # Calculate error and PD terms
        error = float(right_value) - float(left_value)
        
        # Adaptive cruise speed based on line centering
        avg = (left_value + right_value) / 2.0
        if avg > LINE_THRESHOLD + 15:
            cruise = min(self.max_speed, self.base_speed + 4)
        elif avg > LINE_THRESHOLD:
            cruise = self.base_speed
        else:
            cruise = max(self.min_speed, self.base_speed - 8)

        # PD control
        P = self.kp * error
        D = self.kd * ((error - self.last_error) / dt) if dt > 0 else 0.0
        self.last_error = error

        correction = P + D

        # Apply dead zone and clamping
        if abs(error) < DEAD_ZONE:
            correction = 0.0
        correction = max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        # Filter correction
        self.filtered_correction = (CORRECTION_FILTER_ALPHA * self.filtered_correction +
                                    (1.0 - CORRECTION_FILTER_ALPHA) * correction)

        # Compute wheel speeds
        left_speed = cruise - self.filtered_correction
        right_speed = cruise + self.filtered_correction

        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        if right_speed>left_speed:
            self.last_turn_direction="LEFT"
        else:
            self.last_turn_direction="RIGHT"
        return left_speed, right_speed

    def _start_360_search(self):
        """In-place rotation for line search"""
        direction=-1
        if self.last_turn_direction == "RIGHT":
            direction=1

        return direction*SEARCH_SPEED, direction*-SEARCH_SPEED  # Rotate right

    def _continue_360_search(self):
        """Continue in-place rotation"""
        direction=-1
        if self.last_turn_direction == "RIGHT":
            direction=1
        return direction*SEARCH_SPEED, direction*-SEARCH_SPEED

# ===== KEYBOARD & MAIN LOOP =====
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())

    pd_controller = CenterLinePDFollower(KP, KD, BASE_SPEED, MAX_SPEED, MIN_SPEED)

    print("=== ENHANCED CENTER-LINE PD FOLLOWER WITH TIMER RESET ===")
    print("Press 's' start/stop, 'r' reset, 'c' calibrate, 'q' quit")

    running = False
    stop_signal = False

    while not stop_signal:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':
                stop_signal = True
                # Simple program end tone
                sound.beep()
            elif c == 's':
                running = not running
                if running:
                    # Simple start tone
                    sound.beep()
                    print("STARTING LINE FOLLOWING...")
                else:
                    print("STOPPING...")
                    apply_motor_speeds(0.0, 0.0)
            elif c == 'r':
                pd_controller.last_error = 0.0
                pd_controller.last_time = None
                pd_controller.filtered_correction = 0.0
                pd_controller.recovery_state = "NORMAL"
                pd_controller.consecutive_black_detections = 0
                print("Controller reset")
            elif c == 'c':
                # Simple calibration tone
                sound.beep()
                print("\n=== SENSOR CALIBRATION ===")
                print("Place robot so line is BETWEEN sensors and press Enter")
                input()
                centered_L = cl_L.value()
                centered_R = cl_R.value()
                print("Centered - Left: {} Right: {}".format(centered_L, centered_R))

                print("Move robot OFF the line (both sensors on white) and press Enter")
                input()
                off_L = cl_L.value()
                off_R = cl_R.value()
                print("Off line - Left: {} Right: {}".format(off_L, off_R))

                suggested = (min(centered_L, centered_R) + max(off_L, off_R)) / 2.0
                print("Suggested LINE_THRESHOLD: {:.0f}".format(suggested))

        if running and not stop_signal:
            # Read sensors
            try:
                left_value = cl_L.value()
                right_value = cl_R.value()
            except Exception:
                left_value = cl_L.reflected_light_intensity
                right_value = cl_R.reflected_light_intensity

            left_speed, right_speed = pd_controller.compute(left_value, right_value)

            # Send to motors via central function
            applied_l, applied_r = apply_motor_speeds(left_speed, right_speed)

            # Status print
            error = float(right_value) - float(left_value)
            avg = (left_value + right_value) / 2.0
            state = pd_controller.recovery_state
            if state == "TIMER_STRAIGHT":
                time_remaining = LINE_LOSS_TIMEOUT - (time.time() - pd_controller.recovery_start_time)
                state += " ({:.1f}s)".format(max(0, time_remaining))
            
            print("L:{:3d} R:{:3d} | Avg:{:4.1f} | Err:{:6.2f} | ML:{:5.1f} MR:{:5.1f} | State: {}".format(
                int(left_value), int(right_value), avg, error, applied_l, applied_r, state))

        elif not stop_signal:
            apply_motor_speeds(0.0, 0.0)

        time.sleep(SAMPLE_INTERVAL)

    # Exit cleanup
    apply_motor_speeds(0.0, 0.0)
    print("Program terminated")

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    apply_motor_speeds(0.0, 0.0)