#!/usr/bin/env python3
"""
Simple two-sensor line follower for EV3 (no PID)
- Two light/color sensors (left: in4, right: in1)
- Three motors: left, right, auxiliary (OUTPUT_D, OUTPUT_A, OUTPUT_B)
- Bang-bang / threshold control (no P, I, D)
- Includes calibration, motor direction test, sensor checks and keyboard menu

Author: adapted from user-provided PD example
"""

import sys
import select
import tty
import termios
import time
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent

class SpeedySimple:
    """Simple rule-based line follower"""

    def __init__(self):
        # Sensors
        self.cl_L = ColorSensor('in4')   # left sensor
        self.cl_R = ColorSensor('in1')   # right sensor
        self.cl_L.mode = 'COL-REFLECT'
        self.cl_R.mode = 'COL-REFLECT'

        # Motors
        self.mR = LargeMotor(OUTPUT_A)   # Right motor
        self.mL = LargeMotor(OUTPUT_D)   # Left motor
        self.mB = LargeMotor(OUTPUT_B)   # Auxiliary motor (gripper/other)

        # Sound
        self.sound = Sound()

        # Motion parameters
        self.base_speed = 20      # nominal forward speed (percent)
        self.turn_speed = 10      # how much differential to apply when turning
        self.max_speed = 30
        self.min_speed = -self.max_speed

        # Runtime flags & calibration
        self.running = False
        self.white_value = 9.0
        self.black_value = 6.0
        self.threshold = 50.0     # will be set during calibration
        self.left_motor_polarity = -1
        self.right_motor_polarity = -1

        self.sound.beep()
        print("Simple line follower initialized.")

    def read_sensors(self):
        """Return (left, right) reflectance readings"""
        return self.cl_L.value(), self.cl_R.value()

    def stop_all_motors(self):
        """Stop all motors immediately"""
        self.mL.on(SpeedPercent(0))
        self.mR.on(SpeedPercent(0))
        self.mB.on(SpeedPercent(0))

    def limit_speed(self, speed):
        return max(min(speed, self.max_speed), self.min_speed)

    def test_motor_directions(self):
        """Simple motor direction test for debugging physical build"""
        print("\n=== MOTOR DIRECTION TEST ===")
        print("Left motor forward for 1s")
        self.mL.on(SpeedPercent(30 * self.left_motor_polarity))
        time.sleep(1)
        self.mL.on(SpeedPercent(0))

        print("Right motor forward for 1s")
        self.mR.on(SpeedPercent(30 * self.right_motor_polarity))
        time.sleep(1)
        self.mR.on(SpeedPercent(0))

        print("Both forward for 1s")
        self.mL.on(SpeedPercent(30 * self.left_motor_polarity))
        self.mR.on(SpeedPercent(30 * self.right_motor_polarity))
        time.sleep(1)
        self.stop_all_motors()

        print("If robot moved backward, flip motor polarities with 'f' command")
        self.sound.beep()

    def calibrate_sensors(self):
        """Calibrate on white and black similar to original script"""
        print("\n=== SENSOR CALIBRATION ===")
        print("Place robot on WHITE surface and press Enter...")
        input()
        whites = []
        for _ in range(30):
            l, r = self.read_sensors()
            whites.append((l + r) / 2.0)
            time.sleep(0.03)
        self.white_value = sum(whites) / len(whites)
        self.sound.beep()

        print("Place robot on BLACK line and press Enter...")
        input()
        blacks = []
        for _ in range(30):
            l, r = self.read_sensors()
            blacks.append((l + r) / 2.0)
            time.sleep(0.03)
        self.black_value = sum(blacks) / len(blacks)

        # threshold at midpoint between average white and black
        self.threshold = (self.white_value + self.black_value) / 2.0

        print("Calibration results:")
        print("  White avg: {:.1f}".format(self.white_value))
        print("  Black avg: {:.1f}".format(self.black_value))
        print("  Threshold: {:.1f}".format(self.threshold))
        self.sound.beep()

    def auto_tune_simple(self):
        """Very simple auto-tune for speed depending on contrast"""
        contrast = abs(self.white_value - self.black_value)
        if contrast > 50:
            self.base_speed = 40
            self.turn_speed = 30
        elif contrast > 25:
            self.base_speed = 30
            self.turn_speed = 25
        else:
            self.base_speed = 22
            self.turn_speed = 20
        print("Auto-tuned base_speed={}, turn_speed={} (contrast={:.1f})".format(
            self.base_speed, self.turn_speed, contrast))
        self.sound.beep()

    def get_sensor_quality(self):
        try:
            l, r = self.read_sensors()
            ok = (0 <= l <= 100) and (0 <= r <= 100)
            return ok, "L:{:.1f} R:{:.1f}".format(l, r)
        except Exception as e:
            return False, str(e)

    def flip_motor_directions(self):
        self.left_motor_polarity *= -1
        self.right_motor_polarity *= -1
        print("Motor polarities flipped. Left: {}, Right: {}".format(
            self.left_motor_polarity, self.right_motor_polarity))
        self.sound.beep()

    def execute_line_following(self):
        """
        Simple threshold-based logic (bang-bang):
         - If both sensors see white (above threshold): go straight
         - If left sensor sees the line (lower than threshold): turn left
         - If right sensor sees the line: turn right
         - If both see the line: treat as intersection / sharp correction (reverse+turn)
        """
        l_val, r_val = self.read_sensors()

        # Determine booleans: True if sensor sees "dark" (i.e., line)
        left_on_line = l_val < self.threshold
        right_on_line = r_val < self.threshold

        # Default speeds
        left_speed = self.base_speed
        right_speed = self.base_speed

        # Behavior rules
        if not left_on_line and not right_on_line:
            # Both see white -> go straight forward
            left_speed = self.base_speed
            right_speed = self.base_speed
            state = "STRAIGHT"
        elif left_on_line and not right_on_line:
            # Left sees line -> turn left (slow left, speed right)
            left_speed = self.base_speed - self.turn_speed
            right_speed = self.base_speed + 0  # keep right at base to pivot
            state = "TURN LEFT"
        elif right_on_line and not left_on_line:
            # Right sees line -> turn right (slow right, speed left)
            left_speed = self.base_speed + 0
            right_speed = self.base_speed - self.turn_speed
            state = "TURN RIGHT"
        else:
            # Both see line -> likely cross or very dark area
            # simple recovery: back up slightly and turn
            left_speed = -self.base_speed / 2.0
            right_speed = -self.base_speed / 2.0
            # apply small turn while reversing
            if l_val < r_val:
                # left darker -> turn a bit right
                left_speed = -self.base_speed / 2.0
                right_speed = -self.base_speed / 4.0
            else:
                left_speed = -self.base_speed / 4.0
                right_speed = -self.base_speed / 2.0
            state = "BOTH - RECOVER"

        # Apply polarity and limit speeds
        final_left = self.limit_speed(left_speed * self.left_motor_polarity)
        final_right = self.limit_speed(right_speed * self.right_motor_polarity)

        # Send to motors
        self.mL.on(SpeedPercent(final_left))
        self.mR.on(SpeedPercent(final_right))

        # Debug print
        print("L:{:3.0f} R:{:3.0f} | l_val:{:5.1f} r_val:{:5.1f} | thr:{:4.1f} | ML:{:5.1f} MR:{:5.1f} | {}".format(
            left_speed, right_speed, l_val, r_val, self.threshold, final_left, final_right, state))

def setup_keyboard():
    """Enable non-blocking single-character input"""
    def is_data():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return is_data, old_settings

def print_menu():
    print("\n=== SIMPLE SPEEDY LINE FOLLOWER (NO PID) ===")
    print("Commands:")
    print("  c - Calibrate sensors (white/black)")
    print("  a - Auto-tune simple speeds")
    print("  s - Start/Stop line following")
    print("  m - Test motor directions")
    print("  i - Sensor info")
    print("  f - Flip motor directions")
    print("  r - Reset (stop motors)")
    print("  q - Quit")
    print("============================================\n")

def main():
    robot = SpeedySimple()
    is_data, old_settings = setup_keyboard()

    print_menu()
    print("Tip: run 'm' to test motors, then 'c' to calibrate sensors before 's' to start.")

    try:
        while True:
            if is_data():
                cmd = sys.stdin.read(1).lower()
                if cmd == 'q':
                    print("Quitting.")
                    break
                elif cmd == 'c':
                    robot.stop_all_motors()
                    robot.running = False
                    robot.calibrate_sensors()
                elif cmd == 'a':
                    robot.auto_tune_simple()
                elif cmd == 's':
                    robot.running = not robot.running
                    if robot.running:
                        robot.sound.beep()
                        print("Started line following.")
                    else:
                        robot.stop_all_motors()
                        robot.sound.beep()
                        print("Stopped.")
                elif cmd == 'm':
                    robot.stop_all_motors()
                    robot.running = False
                    robot.test_motor_directions()
                elif cmd == 'i':
                    ok, info = robot.get_sensor_quality()
                    print("Sensor info: {}".format(info))
                    if not ok:
                        robot.sound.speak("Sensor problem")
                elif cmd == 'f':
                    robot.flip_motor_directions()
                elif cmd == 'r':
                    robot.stop_all_motors()
                    robot.running = False
                    print("Motors stopped.")
                elif cmd == 'h':
                    print_menu()

            if robot.running:
                robot.execute_line_following()
                time.sleep(0.03)
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print("\nError:", e)
    finally:
        robot.stop_all_motors()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        robot.sound.beep()
        print("Clean exit. Goodbye.")

if __name__ == "__main__":
    main()
