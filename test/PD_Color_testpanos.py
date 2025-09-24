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

        # PD Parameters (will be calibrated)
        self.kp = 0.5          # Proportional gain
        self.kd = 0.8          # Derivative gain
        self.base_speed = 20   # Base speed
        self.max_speed = 80    # Maximum speed limit

        # Control state
        self.last_error = 0
        self.target_value = 0  # Will be set during calibration
        self.running = False

        # Calibration values
        self.white_value = 100
        self.black_value = 0

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
        print(f"Current KP: {self.kp}")
        print(f"Current KD: {self.kd}")
        print(f"Current Base Speed: {self.base_speed}")

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

        print(f"Updated - KP: {self.kp}, KD: {self.kd}, Speed: {self.base_speed}\n")
        self.sound.beep()

    def compute_pd_control(self, left_value, right_value):
        """
        Calculate PD control output
        Embodied reactive control based on sensor difference
        """
        # Calculate error (sensor difference)
        error = left_value - right_value

        # Proportional term - immediate response to error
        proportional = self.kp * error

        # Derivative term - response to rate of change
        derivative = self.kd * (error - self.last_error)
        self.last_error = error

        # Total correction
        correction = proportional + derivative

        # Apply correction to base speed
        left_speed = self.base_speed - correction
        right_speed = self.base_speed + correction

        # Limit speeds to safe range
        left_speed = self.limit_speed(left_speed)
        right_speed = self.limit_speed(right_speed)

        return left_speed, right_speed, error

    def limit_speed(self, speed):
        """Limit speed to safe operational range"""
        return max(min(speed, self.max_speed), -self.max_speed)

    def execute_line_following(self):
        """
        Main line following behavior
        Embodied sensor-motor coupling
        """
        # Read sensors
        left_value, right_value = self.read_sensors()

        # Calculate motor speeds using PD control
        left_speed, right_speed, error = self.compute_pd_control(left_value, right_value)

        # Apply speeds to motors
        self.mL.on(SpeedPercent(left_speed))
        self.mR.on(SpeedPercent(right_speed))

        # Optional: Use gripper motor for stability or additional functionality
        stability_speed = min(abs(left_speed), abs(right_speed)) / 4
        self.mB.on(SpeedPercent(stability_speed))

        # Debug output
        print(f"L:{left_value:3.0f} | R:{right_value:3.0f} | "
              f"Err:{error:5.1f} | ML:{left_speed:5.1f} | MR:{right_speed:5.1f}")

    def auto_tune_parameters(self):
        """
        Adaptive parameter adjustment based on performance
        Embodied learning - robot improves over time
        """
        # Simple adaptive tuning based on sensor range
        sensor_range = abs(self.white_value - self.black_value)

        if sensor_range > 50:  # High contrast environment
            self.kp = 0.3
            self.kd = 0.5
            self.base_speed = 25
        elif sensor_range > 25:  # Medium contrast
            self.kp = 0.5
            self.kd = 0.8
            self.base_speed = 20
        else:  # Low contrast environment
            self.kp = 0.8
            self.kd = 1.2
            self.base_speed = 15

        print(f"Auto-tuned: KP={self.kp}, KD={self.kd}, Speed={self.base_speed}")
        self.sound.beep()

    def get_sensor_quality(self):
        """Check sensor quality and connection"""
        try:
            left, right = self.read_sensors()
            if 0 <= left <= 100 and 0 <= right <= 100:
                return True, f"Sensors OK - L:{left}, R:{right}"
            else:
                return False, f"Sensor values out of range - L:{left}, R:{right}"
        except Exception as e:
            return False, f"Sensor error: {e}"

def setup_keyboard():
    """Setup non-blocking keyboard input"""
    def is_data():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return is_data, old_settings

def print_menu():
    """Display control menu"""
    print("\n=== SPEEDY GONZALES PD CONTROLLER ===")
    print("Commands:")
    print("  c - Calibrate sensors")
    print("  t - Tune PD parameters")
    print("  a - Auto-tune parameters")
    print("  s - Start/Stop line following")
    print("  r - Reset PD controller")
    print("  i - Check sensor info")
    print("  q - Quit")
    print("=====================================\n")

def main():
    """Main control loop with behavior-based architecture"""
    robot = SpeedyGonzalesPD()
    is_data, old_settings = setup_keyboard()

    print_menu()

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
                    print(f"→ {info}")
                    if not ok:
                        robot.sound.speak("Sensor problem")

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
        print(f"\nError occurred: {e}")
    finally:
        # Cleanup
        robot.stop_all_motors()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        robot.sound.beep()
        print("All motors stopped. Goodbye!")

if __name__ == "__main__":
    main()