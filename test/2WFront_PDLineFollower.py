#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Center-Line PD Follower (inversion-safe)
- The black line is between both sensors.
- Error convention and PD gains unchanged.
- Motor output inversion centralized so code works if robot orientation was flipped.
- All prints use .format() (no f-strings).
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
KP = 18.0        # Proportional gain (unchanged) 18
KD = 3.5         # Derivative gain (unchanged) 3.5
BASE_SPEED = 30   # Nominal base forward speed 25
MAX_SPEED = 80    # Absolute maximum allowed speed per wheel 80
MIN_SPEED = 10     # Minimum allowed speed when near line (turning) 10

# Thresholds for line detection (unchanged)
LINE_THRESHOLD = 8       # calibrate to your surface if needed 8
BOTH_BLACK_THRESHOLD = 3 #3
RECOVERY_STRAIGHT_TIME = 1.0
SEARCH_SPEED = 30

# Control extras (unchanged)
CORRECTION_FILTER_ALPHA = 0.65
MAX_CORRECTION = 18.0
DEAD_ZONE = 1.0

# Sampling
SAMPLE_INTERVAL = 0.02

# ===== MOTOR ORIENTATION FLAG =====
# Set this to True if you physically flipped the robot 180 degrees.
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
sound.beep()

# ===== HELPER: apply speeds centrally with inversion & clamping =====
def apply_motor_speeds(left_speed, right_speed, invert=INVERT_MOTORS):
    """
    left_speed, right_speed: desired speeds in -100..100 (%).
    invert: if True multiply both by -1 to compensate flipped robot.
    Returns (applied_left, applied_right) after clamping and inversion.
    """
    # Clamp to allowed motor range (use MAX_SPEED for safety)
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

# ===== CENTER-LINE FOLLOWER CLASS (unchanged control logic except no negation here) =====
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

        # recovery state
        self.recovery_state = "NORMAL"  # NORMAL, STRAIGHT_RECOVERY, SEARCH_360
        self.recovery_start_time = 0.0
        self.search_dir = 1  # pivot preference: 1 => right, -1 => left

    def compute(self, left_value, right_value):
        """
        Main entry:
        - If line lost -> do recovery state machine.
        - If line found -> normal center-line PD control.
        Returns: (left_speed, right_speed)
        """
        # analyze whether the line is present or lost
        if self._is_line_completely_lost(left_value, right_value):
            # recovery state machine
            if self.recovery_state == "NORMAL":
                self.recovery_state = "STRAIGHT_RECOVERY"
                self.recovery_start_time = time.time()
                print("LINE LOST -> starting straight recovery".format())
                return self._straight_recovery()

            elif self.recovery_state == "STRAIGHT_RECOVERY":
                if time.time() - self.recovery_start_time < RECOVERY_STRAIGHT_TIME:
                    return self._straight_recovery()
                else:
                    self.recovery_state = "SEARCH_360"
                    # choose search direction from last error sign if available
                    self.search_dir = 1 if self.last_error >= 0 else -1
                    print("Straight recovery ended -> starting 360 search (dir={})".format(self.search_dir))
                    return self._start_360_search()

            elif self.recovery_state == "SEARCH_360":
                return self._continue_360_search()

        else:
            # line found -> reset recovery and do normal PD control
            if self.recovery_state != "NORMAL":
                print("Line reacquired -> resuming normal PD following".format())
                sound.beep()
            self.recovery_state = "NORMAL"
            return self._normal_center_line_following(left_value, right_value)

    def _is_line_completely_lost(self, left_value, right_value):
        # if both sensors read very bright (far above typical line threshold), we assume lost
        return (left_value > LINE_THRESHOLD + 20) and (right_value > LINE_THRESHOLD + 20)

    def _normal_center_line_following(self, left_value, right_value):
        """
        Core correction:
        - error := right - left
        - P = kp * error
        - D = kd * (error - last_error) / dt
        - correction = P + D
        - filtered_correction = alpha*filtered_prev + (1-alpha)*correction
        - left_speed = cruise - filtered_correction
          right_speed = cruise + filtered_correction
        NOTE: use cruise speed that leaves headroom for correction.
        """
        now = time.time()
        if self.last_time is None:
            dt = SAMPLE_INTERVAL
        else:
            dt = now - self.last_time
            if dt <= 0.0:
                dt = SAMPLE_INTERVAL
        self.last_time = now

        # ERROR SIGN: right - left -> positive means line moved left (need to turn right)
        error = float(right_value) - float(left_value)

        # choose cruise/current speed based on how centered we are (average reading)
        avg = (left_value + right_value) / 2.0
        if avg > LINE_THRESHOLD + 15:
            cruise = min(self.max_speed, self.base_speed + 4)  # more headroom when very centered
        elif avg > LINE_THRESHOLD:
            cruise = self.base_speed
        else:
            cruise = max(self.min_speed, self.base_speed - 8)

        # PD terms (D uses dt)
        P = self.kp * error
        D = 0.0
        D = self.kd * ((error - self.last_error) / dt)

        self.last_error = error

        correction = P + D

        # dead zone to avoid micro-corrections near center
        if abs(error) < DEAD_ZONE:
            correction = 0.0

        # clamp correction to maximum allowed
        correction = max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        # exponential smoothing
        self.filtered_correction = (CORRECTION_FILTER_ALPHA * self.filtered_correction +
                                    (1.0 - CORRECTION_FILTER_ALPHA) * correction)

        # compute wheel speeds (leave headroom; ensure not exceeding MAX_SPEED)
        left_speed = cruise - self.filtered_correction
        right_speed = cruise + self.filtered_correction

        # ensure we don't exceed physical limits (but do NOT invert here)
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)

        return left_speed, right_speed

    def _straight_recovery(self):
        # simple forward for RECOVERY_STRAIGHT_TIME
        return self.base_speed, self.base_speed

    def _start_360_search(self):
        # rotate in place using search_dir
        return SEARCH_SPEED * self.search_dir, -SEARCH_SPEED * self.search_dir

    def _continue_360_search(self):
        return SEARCH_SPEED * self.search_dir, -SEARCH_SPEED * self.search_dir

# ===== KEYBOARD & MAIN LOOP =====
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())

    pd_controller = CenterLinePDFollower(KP, KD, BASE_SPEED, MAX_SPEED, MIN_SPEED)

    print("=== CENTER-LINE PD FOLLOWER (INVERSION SAFE) ===")
    print("Press 's' start/stop, 'r' reset, 'c' calibrate, 'q' quit".format())

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
                    print("STARTING...".format())
                else:
                    sound.beep()
                    print("STOPPING...".format())
                    apply_motor_speeds(0.0, 0.0)
            elif c == 'r':
                pd_controller.last_error = 0.0
                pd_controller.last_time = None
                pd_controller.filtered_correction = 0.0
                pd_controller.recovery_state = "NORMAL"
                print("Controller reset".format())
            elif c == 'c':
                # simple calibration similar to your previous code
                print("\n=== SENSOR CALIBRATION ===".format())
                print("Place robot so line is BETWEEN sensors and press Enter".format())
                input()
                centered_L = cl_L.value()
                centered_R = cl_R.value()
                print("Centered - Left: {} Right: {}".format(centered_L, centered_R))

                print("Move robot OFF the line (both sensors on white) and press Enter".format())
                input()
                off_L = cl_L.value()
                off_R = cl_R.value()
                print("Off line - Left: {} Right: {}".format(off_L, off_R))

                suggested = (min(centered_L, centered_R) + max(off_L, off_R)) / 2.0
                print("Suggested LINE_THRESHOLD: {:.0f}".format(suggested))

        if running and not stop_signal:
            # read sensors
            try:
                left_value = cl_L.value()
                right_value = cl_R.value()
            except Exception:
                left_value = cl_L.reflected_light_intensity
                right_value = cl_R.reflected_light_intensity

            left_speed, right_speed = pd_controller.compute(left_value, right_value)

            # send to motors via central function (handles inversion)
            applied_l, applied_r = apply_motor_speeds(left_speed, right_speed)

            # status print
            error = float(right_value) - float(left_value)
            avg = (left_value + right_value) / 2.0
            print("L:{:3d} R:{:3d} | Avg:{:4.1f} | Err:{:6.2f} | ML:{:5.1f} MR:{:5.1f} | State: {}".format(
                int(left_value), int(right_value), avg, error, applied_l, applied_r, pd_controller.recovery_state))

        elif not stop_signal:
            apply_motor_speeds(0.0, 0.0)

        time.sleep(SAMPLE_INTERVAL)

    # exit cleanup
    sound.beep()
    apply_motor_speeds(0.0, 0.0)
    print("Program terminated".format())

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    apply_motor_speeds(0.0, 0.0)
