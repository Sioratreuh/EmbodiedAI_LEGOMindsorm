#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Evasive line follower (non-PD) using ev3dev2.

Behavior:
 - The black line is expected to be centered between two color sensors.
 - When a sensor detects black, the robot "evades" the line by turning
   in the opposite direction so the line returns to the sensor pair center.
 - Both sensors black -> pivot in place and search until a sensor finds white.
 - Smooth transitions via exponential filtering.

Controls (keyboard, terminal):
 - 's' : toggle start / stop
 - 'r' : reset filters/state
 - 'q' : quit program

All prints use .format() (no f-strings).
"""

import sys
import select
import tty
import termios
import time

from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sound import Sound

# -----------------------
# Configuration / Tunables
# -----------------------
LEFT_COLOR_PORT = 'in4'   # adjust if needed
RIGHT_COLOR_PORT = 'in1'  # adjust if needed
LEFT_MOTOR_PORT = OUTPUT_D
RIGHT_MOTOR_PORT = OUTPUT_A

# Control parameters (tune for your robot)
BASE_SPEED = 30             # base forward speed (percent)
TURN_MAG = 22               # base turn magnitude (percent units)
DEAD_ZONE = 1.0             # small tolerance on sensor difference to go straight
CORNER_CORRECTION = 7       # Make the corner turns sharper after 

# Reflection thresholds (will be replaced by calibrate() if used)
# Lower values indicate darker (black), higher values indicate brighter (white)
LEFT_THRESHOLD_DEFAULT = 18.0
RIGHT_THRESHOLD_DEFAULT = 18.0

# Smoothing for turn value (exponential filter alpha in [0,1]):
# higher alpha = more smoothing (slower response)
TURN_FILTER_ALPHA = 0.65

# Sampling interval (s)
SAMPLE_INTERVAL = 0.03

# Pivot search parameters
PIVOT_SPEED = 25            # rotation wheel speed when pivoting (percent)
PIVOT_DIRECTION = 1         # default pivot direction (1 => right pivot, -1 => left pivot)
PIVOT_MIN_TIME = 0.05       # minimum loop sleep during pivot (s)
CORNER_BOOST_STEP = 3.5     # extra turn % per loop when stuck turning
CORNER_BOOST_MAX = 35       # maximum extra boost
# -----------------------
# Hardware init
# -----------------------
cl_L = ColorSensor(LEFT_COLOR_PORT)
cl_R = ColorSensor(RIGHT_COLOR_PORT)
# try to set reflect mode; if not supported, it's okay
try:
    cl_L.mode = 'COL-REFLECT'
    cl_R.mode = 'COL-REFLECT'
except Exception:

    pass

tank = MoveTank(LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT)
sound = Sound()

# -----------------------
# Utilities
# -----------------------
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def read_reflection(sensor):
    """
    Try common attribute names to read reflected light.
    Returns float value or raises if unavailable.
    """
    # prefer reflected_light_intensity property if available
    if hasattr(sensor, "reflected_light_intensity"):
        return float(sensor.reflected_light_intensity)
    # older ev3dev: use value()
    if hasattr(sensor, "value"):
        return float(sensor.value())
    # fallback
    raise RuntimeError("Cannot read reflection from sensor (unknown API)")

# -----------------------
# Line follower class
# -----------------------
class EvasiveLineFollower:
    """
    Non-PD evasive line follower:
      - Uses two reflectance sensors
      - If left sees black -> steer right, and vice versa
      - If both see black -> pivot & search until one sensor sees white
    """
    def __init__(self,
                 left_sensor,
                 right_sensor,
                 tank_drive,
                 base_speed=BASE_SPEED,
                 turn_mag=TURN_MAG,
                 left_threshold=LEFT_THRESHOLD_DEFAULT,
                 right_threshold=RIGHT_THRESHOLD_DEFAULT,
                 dead_zone=DEAD_ZONE,
                 turn_alpha=TURN_FILTER_ALPHA):
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.tank = tank_drive
        self.base_speed = float(base_speed)
        self.turn_mag = float(turn_mag)
        self.left_threshold = float(left_threshold)
        self.right_threshold = float(right_threshold)
        self.dead_zone = float(dead_zone)
        self.turn_alpha = float(turn_alpha)

        # filter state
        self.filtered_turn = 0.0

        # pivot direction memory (try to pivot in direction of last evasion)
        self.last_turn_sign = 1
        self.turn_streak_sign = 0
        self.turn_streak_count = 0

    def calibrate(self, samples=20, delay=0.02):
        """
        Simple interactive calibration:
         1) place both sensors on white (off the line) and press Enter
         2) place both sensors on black (on the line) and press Enter
        The method then computes mid-threshold per sensor.
        """
        print("Calibration: place both sensors on WHITE (off line) and press Enter".format())
        input()
        wL = 0.0
        wR = 0.0
        for _ in range(samples):
            wL += read_reflection(self.left_sensor)
            wR += read_reflection(self.right_sensor)
            time.sleep(delay)
        wL /= samples
        wR /= samples

        print("Now place both sensors on BLACK (on line) and press Enter".format())
        input()
        bL = 0.0
        bR = 0.0
        for _ in range(samples):
            bL += read_reflection(self.left_sensor)
            bR += read_reflection(self.right_sensor)
            time.sleep(delay)
        bL /= samples
        bR /= samples

        # midpoint thresholds
        self.left_threshold = (wL + bL) / 2.0
        self.right_threshold = (wR + bR) / 2.0

        print("Calibration complete:".format())
        print("  LEFT white={:.1f}, black={:.1f}, threshold={:.1f}".format(wL, bL, self.left_threshold))
        print("  RIGHT white={:.1f}, black={:.1f}, threshold={:.1f}".format(wR, bR, self.right_threshold))

    def _sensor_states(self):
        """Return tuple (left_on_black, right_on_black, left_val, right_val)."""
        L = read_reflection(self.left_sensor)
        R = read_reflection(self.right_sensor)
        left_on_black = (L < self.left_threshold)
        right_on_black = (R < self.right_threshold)
        return left_on_black, right_on_black, L, R

    def _compute_turn(self, left_on_black, right_on_black, L, R):
        desired = 0.0
        sign = 0

        if left_on_black and not right_on_black:
            desired = +self.turn_mag
            sign = +1
            self.last_turn_sign = 1
        elif right_on_black and not left_on_black:
            desired = -self.turn_mag
            sign = -1
            self.last_turn_sign = -1
        else:
            diff = float(R) - float(L)
            if abs(diff) <= self.dead_zone:
                desired = 0.0
                sign = 0
            else:
                sign = 1 if diff > 0 else -1
                desired = clamp(sign * (self.turn_mag * 0.35),
                                -self.turn_mag * 0.35, self.turn_mag * 0.35)

        # --- Streak counter logic ---
        if sign != 0 and sign == self.turn_streak_sign:
            self.turn_streak_count += 1
        else:
            self.turn_streak_sign = sign
            self.turn_streak_count = 0

        # apply boost if turning same way for a while
        if sign != 0 and self.turn_streak_count > 3:  # allow a few loops before boosting
            boost = min(self.turn_streak_count * CORNER_BOOST_STEP, CORNER_BOOST_MAX)
            desired = sign * (abs(desired) + boost)

        # exponential smoothing
        self.filtered_turn = (self.turn_alpha * self.filtered_turn +
                              (1.0 - self.turn_alpha) * desired)
        return self.filtered_turn

    def _pivot_search(self, prefer_direction=1):
        """
        Pivot in place until at least one sensor stops seeing black.
        prefer_direction: 1 -> pivot right, -1 -> pivot left
        The method rotates wheels in opposite directions to turn in place,
        continuously checking sensors and returning when a white is found.
        """
        print("Pivoting to search line (prefer_direction={})".format(prefer_direction))
        # choose wheel signs so that positive prefer_direction rotates robot to the right
        left_speed = PIVOT_SPEED * prefer_direction
        right_speed = -PIVOT_SPEED * prefer_direction
        # start pivoting
        self.tank.on(SpeedPercent(left_speed), SpeedPercent(right_speed))
        # continuously monitor sensors
        try:
            while True:
                left_on_black, right_on_black, L, R = self._sensor_states()
                # stop pivot when at least one sensor sees white (not black)
                if not (left_on_black and right_on_black):
                    # stop motors and exit
                    self.tank.off()
                    print("Pivot finished: left_on_black={}, right_on_black={}".format(left_on_black, right_on_black))
                    return
                # short sleep to avoid busy loop
                time.sleep(PIVOT_MIN_TIME)
        finally:
            # ensure motors off on exit
            self.tank.off()

    def follow_loop(self):
        """
        Continuous follow loop. Call from main program.
        This loop does not return until stopped by external control.
        """
        print("Starting evasive follow loop. Base speed={}, turn_mag={}".format(self.base_speed, self.turn_mag))
        while True:
            left_on_black, right_on_black, L, R = self._sensor_states()

            # Both sensors black -> pivot search (intersection / sharp turn)
            if left_on_black and right_on_black:
                # prefer pivot in last known turn direction
                prefer_dir = self.last_turn_sign
                # perform pivot until one sensor sees white
                self._pivot_search(prefer_dir)
                # after pivot we continue loop (filtered_turn reset may be desirable)
                self.filtered_turn = 0.0
                continue

            # normal cases: compute turn and drive
            turn = self._compute_turn(left_on_black, right_on_black, L, R)
            left_w, right_w = self._apply_drive(turn)

            # status print
            print("L:{:.1f} R:{:.1f} | L_black:{} R_black:{} | turn:{:.2f} | ML:{:.1f} MR:{:.1f}".format(
                L, R, left_on_black, right_on_black, turn, left_w, right_w))

            time.sleep(SAMPLE_INTERVAL)

# -----------------------
# Main program and controls
# -----------------------
def main():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    follower = EvasiveLineFollower(cl_L, cl_R, tank,
                                   base_speed=BASE_SPEED,
                                   turn_mag=TURN_MAG,
                                   left_threshold=LEFT_THRESHOLD_DEFAULT,
                                   right_threshold=RIGHT_THRESHOLD_DEFAULT,
                                   dead_zone=DEAD_ZONE,
                                   turn_alpha=TURN_FILTER_ALPHA)

    running = False
    stop_program = False

    print("Evasive Line Follower (ev3dev2)".format())
    print("Controls: 's' start/stop, 'r' reset, 'c' calibrate, 'q' quit".format())

    try:
        while not stop_program:
            if is_data():
                ch = sys.stdin.read(1)
                if ch == 'q':
                    stop_program = True
                    break
                elif ch == 's':
                    running = not running
                    if running:
                        sound.beep()
                        print("Starting line follower...".format())
                    else:
                        sound.beep()
                        follower.tank.off()
                        print("Stopped.".format())
                elif ch == 'r':
                    follower.filtered_turn = 0.0
                    follower.last_turn_sign = 1
                    print("Filters/state reset.".format())
                elif ch == 'c':
                    follower.calibrate()
                    print("Calibration applied.".format())

            if running:
                # run one follow iteration (blocking inside follow_loop is not used;
                # instead call the helper _sensor_states/_compute/_apply here)
                left_on_black, right_on_black, L, R = follower._sensor_states()

                if left_on_black and right_on_black:
                    follower._pivot_search(follower.last_turn_sign)
                    follower.filtered_turn = 0.0
                    continue

                turn = follower._compute_turn(left_on_black, right_on_black, L, R)
                left_w, right_w = follower._apply_drive(turn)

                print("L:{:.1f} R:{:.1f} | L_black:{} R_black:{} | turn:{:.2f} | ML:{:.1f} MR:{:.1f}".format(
                    L, R, left_on_black, right_on_black, turn, left_w, right_w))

            else:
                follower.tank.off()

            time.sleep(SAMPLE_INTERVAL)

    finally:
        # cleanup
        sound.beep()
        tank.off()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Program terminated.".format())

if __name__ == "__main__":
    main()
