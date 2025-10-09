#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced 3-Sensor PD Line Follower (Upgraded 2025)
- Improved PD control with filtered derivative and adaptive correction clamp
- Multi-sample calibration for robust thresholds
- Ziegler–Nichols tuning helper built in
- Retains keyboard control, recovery logic, and diagnostic display
"""

import sys
import select
import tty
import termios
import time
from statistics import mean

from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent

# ===== DEFAULT CONFIGURATION =====
KP_MIDDLE = 15
KD_MIDDLE = 2.5
KP_SIDES = 12
KD_SIDES = 2.0

BASE_SPEED = 30
MAX_SPEED = 80
MIN_SPEED = 15

MIDDLE_THRESHOLD = 9.4
SIDE_THRESHOLD = 7.9

LINE_LOSS_TIMEOUT = 0.7
CURVE_DETECTION_THRESHOLD = 15
SHARP_CURVE_THRESHOLD = 25
SEARCH_SPEED = 30

CORRECTION_FILTER_ALPHA = 0.7
MAX_CORRECTION = 20.0
DEAD_ZONE = 1.0
SAMPLE_INTERVAL = 0.02
INVERT_MOTORS = True

# ===== SENSOR & MOTOR SETUP =====
cl_L = ColorSensor('in4')
cl_R = ColorSensor('in1')
cl_M = ColorSensor('in3')

try:
    for s in (cl_L, cl_R, cl_M):
        s.mode = 'COL-REFLECT'
except Exception:
    pass

mL = LargeMotor(OUTPUT_D)
mR = LargeMotor(OUTPUT_A)

sound = Sound()
sound.beep()


# ===== HELPER FUNCTIONS =====
def apply_motor_speeds(left_speed, right_speed, invert=INVERT_MOTORS):
    l = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    r = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    if invert:
        l, r = -l, -r
    mL.on(SpeedPercent(l))
    mR.on(SpeedPercent(r))
    return l, r


def auto_calibrate_sensor(sensor, name, samples=20):
    print("\nCalibrating {} sensor...".format(name))
    black_values, white_values = [], []

    input("Place {} sensor on BLACK line and press Enter...".format(name))
    for _ in range(samples):
        black_values.append(sensor.value())
        time.sleep(0.05)

    input("Place {} sensor on WHITE area and press Enter...".format(name))
    for _ in range(samples):
        white_values.append(sensor.value())
        time.sleep(0.05)

    black_avg = mean(black_values)
    white_avg = mean(white_values)
    threshold = (black_avg + white_avg) / 2.0
    print("{}: Black={:.1f}, White={:.1f}, Threshold={:.1f}".format(name, black_avg, white_avg, threshold))
    return threshold


# ===== PD FOLLOWER CLASS =====
class EnhancedThreeSensorFollower:
    def __init__(self, kp_middle, kd_middle, kp_sides, kd_sides, base_speed, max_speed, min_speed):
        self.kp_middle = float(kp_middle)
        self.kd_middle = float(kd_middle)
        self.kp_sides = float(kp_sides)
        self.kd_sides = float(kd_sides)
        self.base_speed = float(base_speed)
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)

        self.last_error_middle = 0.0
        self.last_error_sides = 0.0
        self.last_time = None
        self.filtered_correction = 0.0
        self.derivative_filter = 0.0
        self.last_middle_value = 0.0

        self.recovery_state = "NORMAL"
        self.recovery_start_time = 0.0
        self.last_known_correction = 0.0
        self.consecutive_black_detections = 0
        self.last_turn_direction = "RIGHT"

        self.curve_state = "STRAIGHT"
        self.curve_direction = "NONE"

    def suggest_gains(self, Ku, Tu):
        kp = 0.8 * Ku
        kd = kp * (Tu / 8.0)
        print("Suggested PD Gains → Kp: {:.2f}, Kd: {:.2f}".format(kp, kd))
        return kp, kd

    def compute(self, left_value, right_value, middle_value):
        self._detect_curve(left_value, right_value, middle_value)
        line_lost = self._is_line_lost(left_value, right_value, middle_value)

        if line_lost:
            return self._handle_line_loss(left_value, right_value, middle_value)
        else:
            self.recovery_state = "NORMAL"
            self.consecutive_black_detections = 0
            return self._adaptive_control(left_value, right_value, middle_value)

    def _detect_curve(self, left_value, right_value, middle_value):
        side_diff = right_value - left_value
        abs_diff = abs(side_diff)
        if side_diff > CURVE_DETECTION_THRESHOLD:
            self.curve_direction = "LEFT"
        elif side_diff < -CURVE_DETECTION_THRESHOLD:
            self.curve_direction = "RIGHT"
        else:
            self.curve_direction = "NONE"

        if abs_diff > SHARP_CURVE_THRESHOLD:
            self.curve_state = "SHARP_CURVE"
        elif abs_diff > CURVE_DETECTION_THRESHOLD:
            self.curve_state = "GENTLE_CURVE"
        else:
            self.curve_state = "STRAIGHT"

    def _adaptive_control(self, left_value, right_value, middle_value):
        if self.curve_state == "STRAIGHT":
            # Usamos la corrección del sensor central directamente
            m_corr = self._calculate_middle_correction(middle_value)
            return self._apply_correction(m_corr)
        elif self.curve_state == "GENTLE_CURVE":
            m_corr = self._calculate_middle_correction(middle_value)
            s_corr = self._calculate_side_correction(left_value, right_value)
            blended = 0.7 * m_corr + 0.3 * s_corr
            return self._apply_correction(blended)
        else:
            s_corr = self._calculate_side_correction(left_value, right_value)
            return self._apply_correction(s_corr)


    def _calculate_middle_correction(self, middle_value):
        now = time.time()
        dt = SAMPLE_INTERVAL if not self.last_time else max(time.time() - self.last_time, SAMPLE_INTERVAL)
        self.last_time = now

        error = float(MIDDLE_THRESHOLD - middle_value)

        if abs(error) < 2.0:
            self.current_speed = min(self.max_speed, self.base_speed + 5)
        elif abs(error) < 5.0:
            self.current_speed = self.base_speed
        else:
            self.current_speed = max(self.min_speed, self.base_speed - 8)

        self.derivative_filter = 0.8 * self.derivative_filter + 0.2 * ((self.last_middle_value - middle_value) / dt)
        self.last_middle_value = middle_value

        P = self.kp_middle * error
        D = self.kd_middle * self.derivative_filter
        correction = P + D

        if abs(error) < DEAD_ZONE:
            correction = 0.0

        max_corr = MAX_CORRECTION * (self.current_speed / self.max_speed)
        correction = max(min(correction, max_corr), -max_corr)

        self.filtered_correction = (CORRECTION_FILTER_ALPHA * self.filtered_correction +
                                   (1 - CORRECTION_FILTER_ALPHA) * correction)
        return self.filtered_correction
    
    def _calculate_side_correction(self, left_value, right_value):
        now = time.time()
        dt = SAMPLE_INTERVAL if not self.last_time else max(time.time() - self.last_time, SAMPLE_INTERVAL)
        error = float(right_value - left_value)
        P = self.kp_sides * error
        D = self.kd_sides * ((error - self.last_error_sides) / dt)
        self.last_error_sides = error
        correction = P + D
        if abs(error) < DEAD_ZONE:
            correction = 0.0
        return max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

    def _apply_correction(self, correction):
        left_speed = self.current_speed - correction
        right_speed = self.current_speed + correction
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        if right_speed > left_speed:
            self.last_turn_direction = "LEFT"
        else:
            self.last_turn_direction = "RIGHT"
        return left_speed, right_speed

    def _handle_line_loss(self, left, right, middle):
        current_time = time.time()
        if self.recovery_state == "NORMAL":
            self.recovery_state = "TIMER_STRAIGHT"
            self.recovery_start_time = current_time
            self.last_known_correction = self.filtered_correction
            print("LINE LOST -> Holding last correction")
            return self._timer_straight_recovery()
        elif self.recovery_state == "TIMER_STRAIGHT":
            if current_time - self.recovery_start_time < LINE_LOSS_TIMEOUT:
                return self._timer_straight_recovery()
            else:
                self.recovery_state = "SEARCH_360"
                print("Timer expired -> searching line")
                return self._start_360_search()
        elif self.recovery_state == "SEARCH_360":
            return self._continue_360_search()

    def _is_line_lost(self, l, r, m):
        return (l > SIDE_THRESHOLD) and (r > SIDE_THRESHOLD) and (m > MIDDLE_THRESHOLD)

    def _timer_straight_recovery(self):
        ls = self.base_speed - self.last_known_correction
        rs = self.base_speed + self.last_known_correction
        return max(min(ls, self.max_speed), -self.max_speed), max(min(rs, self.max_speed), -self.max_speed)

    def _start_360_search(self):
        d = -1 if self.last_turn_direction == "RIGHT" else 1
        return d * SEARCH_SPEED, -d * SEARCH_SPEED

    def _continue_360_search(self):
        d = -1 if self.last_turn_direction == "RIGHT" else 1
        return d * SEARCH_SPEED, -d * SEARCH_SPEED


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())

    controller = EnhancedThreeSensorFollower(KP_MIDDLE, KD_MIDDLE, KP_SIDES, KD_SIDES, BASE_SPEED, MAX_SPEED, MIN_SPEED)

    print("=== ENHANCED 3-SENSOR PD LINE FOLLOWER (Upgraded) ===")
    print("Press 's' start/stop, 'r' reset, 'c' calibrate, 'z' Z-N helper, 'q' quit")

    running = False
    stop_signal = False

    while not stop_signal:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':
                stop_signal = True
                sound.beep()
            elif c == 's':
                running = not running
                sound.beep()
                print("STARTING..." if running else "STOPPING...")
                if not running:
                    apply_motor_speeds(0, 0)
            elif c == 'r':
                controller.last_error_middle = 0
                controller.last_error_sides = 0
                controller.filtered_correction = 0
                controller.recovery_state = "NORMAL"
                print("Controller reset.")
            elif c == 'c':
                sound.beep()
                print("\n=== AUTO CALIBRATION ===")
                MIDDLE_THRESHOLD = auto_calibrate_sensor(cl_M, "MIDDLE")
                left_th = auto_calibrate_sensor(cl_L, "LEFT")
                right_th = auto_calibrate_sensor(cl_R, "RIGHT")
                SIDE_THRESHOLD = (left_th + right_th) / 2
                print("\nSuggested thresholds → MIDDLE:{:.1f}, SIDES:{:.1f}".format(MIDDLE_THRESHOLD, SIDE_THRESHOLD))
            elif c == 'z':
                sound.beep()
                print("\n=== ZIEGLER–NICHOLS TUNING ===")
                Ku = float(input("Enter ultimate gain (Ku): "))
                Tu = float(input("Enter oscillation period (Tu seconds): "))
                controller.suggest_gains(Ku, Tu)

        if running and not stop_signal:
            try:
                lv, rv, mv = cl_L.value(), cl_R.value(), cl_M.value()
            except Exception:
                lv, rv, mv = cl_L.reflected_light_intensity, cl_R.reflected_light_intensity, cl_M.reflected_light_intensity

            ls, rs = controller.compute(lv, rv, mv)
            al, ar = apply_motor_speeds(ls, rs)
            state = controller.recovery_state
            if state == "TIMER_STRAIGHT":
                t_rem = LINE_LOSS_TIMEOUT - (time.time() - controller.recovery_start_time)
                state = state + " ({:.1f}s)".format(max(0, t_rem))
            print("L:{:3d} R:{:3d} M:{:3d} | Curve:{} {} | ML:{:5.1f} MR:{:5.1f} | State:{}".format(
                lv, rv, mv, controller.curve_state, controller.curve_direction, al, ar, state))

        else:
            apply_motor_speeds(0, 0)
        time.sleep(SAMPLE_INTERVAL)

    apply_motor_speeds(0, 0)
    print("Program terminated.")

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    apply_motor_speeds(0, 0)
