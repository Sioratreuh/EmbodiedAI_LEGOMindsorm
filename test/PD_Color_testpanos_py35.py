#!/usr/bin/env python3
"""
SDU - Embodied AI Project: Speedy Gonzales
- Jose Angel Huerta Rios
- Andres De Pool
- Panagiotis Nikolaodis

Improved PD Controller Line Follower with Embodied AI Principles
- No hardcoded values
- Simple, modular functions
- Adaptive parameter tuning
"""

import sys
import select
import tty
import termios
import time
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent

class SpeedyGonzalesPD:
    """Embodied AI PD Controller for Line Following"""

    def __init__(self):
        """Initialize robot with adaptive parameters"""
        # Sensors
        self.cl_L = ColorSensor('in4')
        self.cl_R = ColorSensor('in1')
        self.cl_L.mode = 'COL-REFLECT'
        self.cl_R.mode = 'COL-REFLECT'

        # Motors
        self.mR = LargeMotor(OUTPUT_A)  # Right motor
        self.mL = LargeMotor(OUTPUT_D)  # Left motor
        self.mB = LargeMotor(OUTPUT_B)  # Gripper/auxiliary motor

        # Sound feedback
        self.sound = Sound()

        # PD Parameters - TUNED FOR BETTER TURNING
        self.kp = 1.5          # Increased proportional gain for stronger response
        self.kd = 0.3          # Reduced derivative to avoid oscillation
        self.base_speed = 30   # Base speed - positive for forward
        self.max_speed = 80    # Maximum speed limit
        self.min_speed = -60   # Minimum speed (for reverse)

        # Control state
        self.last_error = 0
        self.target_value = 0  # Will be set during calibration
        self.running = False

        # Calibration values
        self.white_value = 100
        self.black_value = 0

        # Motor direction multipliers (adjust if robot moves backwards)
        self.left_motor_polarity = 1   # Change to -1 if left motor goes wrong direction
        self.right_motor_polarity = 1  # Change to -1 if right motor goes wrong direction

        self.sound.beep()
        print("Robot initialized successfully!")

    def read_sensors(self):
        """Read both color sensors"""
        left = self.cl_L.value()
        right = self.cl_R.value()
        return left, right

    def stop_all_motors(self):
        """Emergency stop for all motors"""
        self.mL.on(SpeedPercent(0))
        self.mR.on(SpeedPercent(0))
        self.mB.on(SpeedPercent(0))

    def test_motor_directions(self):
        """Test motor directions to ensure proper movement"""
        print("\n=== MOTOR DIRECTION TEST ===")
        print("Testing motor directions...")

        # Test left motor forward
        print("Left motor forward for 1 second...")
        self.mL.on(SpeedPercent(30))
        time.sleep(1)
        self.mL.on(SpeedPercent(0))

        # Test right motor forward
        print("Right motor forward for 1 second...")
        self.mR.on(SpeedPercent(30))
        time.sleep(1)
        self.mR.on(SpeedPercent(0))

        # Test both motors forward
        print("Both motors forward for 1 second...")
        self.mL.on(SpeedPercent(30))
        self.mR.on(SpeedPercent(30))
        time.sleep(1)
        self.stop_all_motors()

        print("If robot didn't move forward properly, adjust motor polarities")
        print("Motor direction test complete!\n")
        self.sound.beep()

    def calibrate_sensors(self):
        """
        Calibrate sensors on white and black surfaces
        Embodied learning - robot adapts to environment
        """
        print("\n=== SENSOR CALIBRATION ===")

        # Calibrate white surface
        print("Place robot on WHITE surface and press Enter...")
        input()
        white_readings = []
        for i in range(30):
            left, right = self.read_sensors()
            white_readings.append((left + right) / 2)
            time.sleep(0.05)
        self.white_value = sum(white_readings) / len(white_readings)
        self.sound.beep()

        # Calibrate black line
        print("Place robot on BLACK line and press Enter...")
        input()
        black_readings = []
        for i in range(30):
            left, right = self.read_sensors()
            black_readings.append((left + right) / 2)
            time.sleep(0.05)
        self.black_value = sum(black_readings) / len(black_readings)

        # Calculate target (edge of line)
        self.target_value = (self.white_value + self.black_value) / 2

        print("White: {:.1f}".format(self.white_value))
        print("Black: {:.1f}".format(self.black_value))
        print("Target: {:.1f}".format(self.target_value))
        self.sound.beep()
        print("Calibration complete!\n")

    def tune_pd_parameters(self):
        """Interactive PD parameter tuning"""
        print("\n=== PD PARAMETER TUNING ===")
        print("Current KP: {}".format(self.kp))
        print("Current KD: {}".format(self.kd))
        print("Current Base Speed: {}".format(self.base_speed))

        try:
            kp_input = input("Enter new KP (or press Enter to keep current): ")
            if kp_input.strip():
                self.kp = float(kp_input)

            kd_input = input("Enter new KD (or press Enter to keep current): ")
            if kd_input.strip():
                self.kd = float(kd_input)

            speed_input = input("Enter new Base Speed (or press Enter to keep current): ")
            if speed_input.strip():
                self.base_speed = int(speed_input)

        except ValueError:
            print("Invalid input, keeping current values")

        print("Updated - KP: {}, KD: {}, Speed: {}\n".format(self.kp, self.kd, self.base_speed))
        self.sound.beep()

    def compute_pd_control(self, left_value, right_value):
        """
        FIXED PD control calculation for proper turning
        """
        # Calculate error (difference between sensors)
        # Positive error = line is more to the left (left sensor sees darker)
        # Negative error = line is more to the right (right sensor sees darker)
        error = left_value - right_value

        # Proportional term - immediate response to current error
        proportional = self.kp * error

        # Derivative term - response to rate of change of error
        derivative = self.kd * (error - self.last_error)
        self.last_error = error

        # Total correction
        correction = proportional + derivative

        # Apply correction to motors
        # When error is positive (line to left), we need to turn left:
        # - Slow down left motor, speed up right motor
        # When error is negative (line to right), we need to turn right:
        # - Speed up left motor, slow down right motor

        left_speed = self.base_speed - correction
        right_speed = self.base_speed + correction

        # Limit speeds to safe operating range
        left_speed = self.limit_speed(left_speed)
        right_speed = self.limit_speed(right_speed)

        return left_speed, right_speed, error, correction

    def limit_speed(self, speed):
        """Limit speed to safe operational range"""
        return max(min(speed, self.max_speed), self.min_speed)

    def execute_line_following(self):
        """
        Main line following behavior with improved turning
        """
        # Read sensors
        left_value, right_value = self.read_sensors()

        # Calculate motor speeds using FIXED PD control
        left_speed, right_speed, error, correction = self.compute_pd_control(left_value, right_value)

        # Apply motor polarity corrections
        final_left_speed = left_speed * self.left_motor_polarity
        final_right_speed = right_speed * self.right_motor_polarity

        # Apply speeds to motors
        self.mL.on(SpeedPercent(final_left_speed))
        self.mR.on(SpeedPercent(final_right_speed))

        # Optional: Use gripper motor - removed for debugging
        # self.mB.on(SpeedPercent(0))

        # Enhanced debug output
        direction = ""
        if abs(error) < 5:
            direction = "STRAIGHT"
        elif error > 5:
            direction = "TURN LEFT"
        elif error < -5:
            direction = "TURN RIGHT"

        print("L:{:3.0f} R:{:3.0f} | Err:{:6.1f} Corr:{:6.1f} | ML:{:5.1f} MR:{:5.1f} | {}".format(
            left_value, right_value, error, correction, final_left_speed, final_right_speed, direction))

    def auto_tune_parameters(self):
        """
        Improved adaptive parameter adjustment
        """
        # Simple adaptive tuning based on sensor range
        sensor_range = abs(self.white_value - self.black_value)

        if sensor_range > 50:  # High contrast environment
            self.kp = 1.2
            self.kd = 0.2
            self.base_speed = 35
        elif sensor_range > 25:  # Medium contrast
            self.kp = 1.5
            self.kd = 0.3
            self.base_speed = 30
        else:  # Low contrast environment
            self.kp = 2.0
            self.kd = 0.4
            self.base_speed = 25

        print("Auto-tuned: KP={}, KD={}, Speed={}".format(self.kp, self.kd, self.base_speed))
        self.sound.beep()

    def get_sensor_quality(self):
        """Check sensor quality and connection"""
        try:
            left, right = self.read_sensors()
            if 0 <= left <= 100 and 0 <= right <= 100:
                return True, "Sensors OK - L:{}, R:{}".format(left, right)
            else:
                return False, "Sensor values out of range - L:{}, R:{}".format(left, right)
        except Exception as e:
            return False, "Sensor error: {}".format(e)

    def flip_motor_directions(self):
        """Flip motor direction polarities if robot moves backwards"""
        self.left_motor_polarity *= -1
        self.right_motor_polarity *= -1
        print("Motor directions flipped!")
        self.sound.beep()

def setup_keyboard():
    """Setup non-blocking keyboard input"""
    def is_data():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return is_data, old_settings

def print_menu():
    """Display control menu"""
    print("\n=== SPEEDY GONZALES PD CONTROLLER (FIXED) ===")
    print("Commands:")
    print("  c - Calibrate sensors")
    print("  t - Tune PD parameters")
    print("  a - Auto-tune parameters")
    print("  s - Start/Stop line following")
    print("  r - Reset PD controller")
    print("  i - Check sensor info")
    print("  m - Test motor directions")
    print("  f - Flip motor directions")
    print("  q - Quit")
    print("===============================================\n")

def main():
    """Main control loop with behavior-based architecture"""
    robot = SpeedyGonzalesPD()
    is_data, old_settings = setup_keyboard()

    print_menu()
    print("IMPORTANT: Run 'm' to test motor directions first!")
    print("Then run 'c' to calibrate sensors before starting!")

    try:
        while True:
            # Check for keyboard commands (non-blocking)
            if is_data():
                command = sys.stdin.read(1).lower()

                if command == 'q':
                    print("\nShutting down Speedy Gonzales...")
                    break

                elif command == 'c':
                    robot.stop_all_motors()
                    robot.running = False
                    robot.calibrate_sensors()

                elif command == 't':
                    robot.stop_all_motors()
                    robot.running = False
                    robot.tune_pd_parameters()

                elif command == 'a':
                    robot.auto_tune_parameters()

                elif command == 's':
                    robot.running = not robot.running
                    if robot.running:
                        robot.sound.beep()
                        print("→ STARTING line following...")
                    else:
                        robot.stop_all_motors()
                        robot.sound.beep()
                        print("→ STOPPED")

                elif command == 'r':
                    robot.last_error = 0
                    robot.sound.beep()
                    print("→ PD controller reset")

                elif command == 'i':
                    ok, info = robot.get_sensor_quality()
                    print("→ {}".format(info))
                    if not ok:
                        robot.sound.speak("Sensor problem")

                elif command == 'm':
                    robot.stop_all_motors()
                    robot.running = False
                    robot.test_motor_directions()

                elif command == 'f':
                    robot.flip_motor_directions()

                elif command == 'h':
                    print_menu()

            # Execute behaviors
            if robot.running:
                robot.execute_line_following()
                time.sleep(0.02)  # Control loop timing
            else:
                time.sleep(0.1)   # Idle timing

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print("\nError occurred: {}".format(e))
    finally:
        # Cleanup
        robot.stop_all_motors()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        robot.sound.beep()
        print("All motors stopped. Goodbye!")

if __name__ == "__main__":
    main()