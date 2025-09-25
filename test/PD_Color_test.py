#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SDU - Embedded AI Project: Speedy Gonzales
PD Controller with Speed & Turn Limits and "evade-the-line" logic.

Authors:
 - Jose Angel Huerta Rios
 - Andres De Pool
 - Panagiotis Nikolaodis

Notes:
 - Logic: the black line goes in the middle of both sensors. When a sensor
   detects black, the robot "evades" the line by turning in the opposite direction to
   re-center it.
 - No f-strings are used; all prints use .format().
"""

import sys
import select
import tty
import termios
import time

from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent

# ===========================
# ===== CONFIGURATION =======
# ===========================

# PD params (adjust according to your robot and track)
KP = 5.0           # proportional gain
KD = 4.0           # derivative gain
DEAD_ZONE = 0.5    # error threshold below which the robot goes straight

# Speed / turn limits
MAX_SPEED = 40                 # maximum absolute speed per wheel (percentage)
MAX_TURN_ADJ = 15.0            # maximum correction (in equivalent "speed" units)
CRUISE_SPEED = MAX_SPEED - (MAX_TURN_ADJ / 2.0)  # base speed on straight (adjusted)
# Note: CRUISE_SPEED chosen to leave margin for lateral correction.

# Smoothing
CORRECTION_FILTER_ALPHA = 0.6  # 0..1 (1=no filter, 0=only history)

# Sampling
SAMPLE_INTERVAL = 0.02  # seconds between iterations

# Ports (adjust if your sensors/motors are on other ports)
LEFT_COLOR_PORT = 'in4'   # left sensor
RIGHT_COLOR_PORT = 'in1'  # right sensor
LEFT_MOTOR_PORT = OUTPUT_D
RIGHT_MOTOR_PORT = OUTPUT_A

# ===========================
# ===== INITIALIZATION ======
# ===========================
cl_L = ColorSensor(LEFT_COLOR_PORT)
cl_R = ColorSensor(RIGHT_COLOR_PORT)
try:
    cl_L.mode = 'COL-REFLECT'
    cl_R.mode = 'COL-REFLECT'
except Exception:
    # on some firmwares mode assignment may fail; not critical
    pass

mL = LargeMotor(LEFT_MOTOR_PORT)   # left motor
mR = LargeMotor(RIGHT_MOTOR_PORT)  # right motor

sound = Sound()
sound.beep()

# ===========================
# ===== UTILITIES ==========
# ===========================
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

# ===========================
# ===== PD CONTROLLER =======
# ===========================
class SmoothPD:
    """
    PD controller with filtered correction and limiting.
    Assumes 'error' = right - left (sensor readings)
    - If left << right  => positive error => reduce left speed,
      increase right speed => turn right (evade line to the left).
    """
    def __init__(self, kp, kd, cruise_speed, max_turn, dead_zone, alpha=0.6):
        self.kp = kp
        self.kd = kd
        self.cruise_speed = cruise_speed
        self.max_turn = max_turn
        self.dead_zone = dead_zone
        self.alpha = alpha
        self.last_error = 0.0
        self.last_time = None
        self.filtered_correction = 0.0

    def reset(self):
        self.last_error = 0.0
        self.last_time = None
        self.filtered_correction = 0.0

    def compute(self, left_read, right_read):
        """
        left_read, right_read: raw reflectance readings (sensor value()).
        Returns (left_speed, right_speed) already limited to range [-MAX_SPEED, MAX_SPEED].
        """
        now = time.time()
        if self.last_time is None:
            dt = None
        else:
            dt = now - self.last_time
        self.last_time = now

        # error: simple difference (R - L)
        error = float(right_read) - float(left_read)

        # Dead zone: if error is small, consider centered and go straight
        if abs(error) < self.dead_zone:
            # reset derivative state to avoid bump when error returns
            self.last_error = error
            self.filtered_correction = 0.0
            return self._apply_speed_limits(self.cruise_speed, self.cruise_speed, error)

        # P term
        P = self.kp * error

        # D term: error derivative according to dt; if dt is None or too small, D=0
        D = 0.0
        if dt is not None and dt > 1e-6:
            D = self.kd * ((error - self.last_error) / dt)
        else:
            D = 0.0

        self.last_error = error

        # Unfiltered correction
        correction = P + D

        # Clamp correction
        correction = clamp(correction, -self.max_turn, self.max_turn)

        # Exponential filtering for smoothing
        self.filtered_correction = self.alpha * self.filtered_correction + (1.0 - self.alpha) * correction

        # Apply correction: decrease left wheel and increase right wheel
        left_speed = self.cruise_speed - self.filtered_correction
        right_speed = self.cruise_speed + self.filtered_correction

        return self._apply_speed_limits(left_speed, right_speed, error)

    def _apply_speed_limits(self, l, r, error_snapshot):
        # Ensure speeds don't exceed MAX_SPEED and aren't less than -MAX_SPEED
        l_clamped = clamp(l, -MAX_SPEED, MAX_SPEED)
        r_clamped = clamp(r, -MAX_SPEED, MAX_SPEED)
        # If speeds are out of range due to saturation, we could scale
        # here if we wanted to preserve the ratio; for simplicity we clamp directly.
        return l_clamped, r_clamped, error_snapshot

# ===========================
# ===== MAIN PROGRAM ========
# ===========================
def main():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    pd = SmoothPD(KP, KD, CRUISE_SPEED, MAX_TURN_ADJ, DEAD_ZONE, CORRECTION_FILTER_ALPHA)

    running = False
    stop_program = False

    print("=== PD EVASIVE LINE FOLLOWER (no f-strings) ===")
    print("KP={}, KD={}, DEAD_ZONE={}".format(KP, KD, DEAD_ZONE))
    print("MAX_SPEED={}, CRUISE_SPEED={}, MAX_TURN_ADJ={}".format(MAX_SPEED, CRUISE_SPEED, MAX_TURN_ADJ))
    print("Controls: 's' start/stop, 'r' reset PD, 'q' quit")
    print("==============================================")

    try:
        while not stop_program:
            # non-blocking keyboard
            if is_data():
                ch = sys.stdin.read(1)
                if ch == 'q':
                    stop_program = True
                    break
                elif ch == 's':
                    running = not running
                    if running:
                        sound.beep()
                        print("STARTING...".format())
                    else:
                        sound.beep()
                        print("STOPPING...".format())
                        # Turn off motors immediately
                        mL.on(SpeedPercent(0))
                        mR.on(SpeedPercent(0))
                elif ch == 'r':
                    pd.reset()
                    print("PD controller reset".format())

            if running:
                # Sensor readings: use .value() for compatibility with various versions
                try:
                    L = cl_L.value()
                    R = cl_R.value()
                except Exception:
                    # fallback to attribute if API exposes it that way
                    L = cl_L.reflected_light_intensity
                    R = cl_R.reflected_light_intensity

                # Calculate speeds with PD
                left_v, right_v, err = pd.compute(L, R)

                # Send to motors (SpeedPercent requires values in -100..100)
                mL.on(SpeedPercent(left_v))
                mR.on(SpeedPercent(right_v))

                # Status print (using .format)
                print("L:{:.1f} R:{:.1f} | err:{:.2f} | ML:{:.1f} MR:{:.1f}".format(
                    float(L), float(R), float(err), float(left_v), float(right_v)
                ))

            else:
                # if not running, ensure motors are off
                mL.on(SpeedPercent(0))
                mR.on(SpeedPercent(0))

            time.sleep(SAMPLE_INTERVAL)

    finally:
        # cleanup
        sound.beep()
        mL.on(SpeedPercent(0))
        mR.on(SpeedPercent(0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Program terminated".format())

if __name__ == "__main__":
    main()