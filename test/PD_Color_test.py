#!/usr/bin/env python3
'''
Center-Line PD Follower with 360° Recovery
- Python 3.5.3 Compatible (no f-strings)
- Line stays between the two sensors
- Go straight for 1s when lost, then 360° search
- Two-motor setup only
- Embodied AI reactive architecture
'''
import sys
import select
import tty
import termios
import time
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent

# ===== CENTER-LINE PD CONFIGURATION =====
KP = 3.0          # Proportional gain for center-line following
KD = 2.5          # Derivative gain to prevent oscillation
BASE_SPEED = 40   # Positive speed (forward)
MAX_SPEED = 60    # Maximum speed for straight sections
MIN_SPEED = 25    # Minimum speed for turns

# Thresholds for center-line following
LINE_THRESHOLD = 50      # Above this = white surface, below = black line
BOTH_BLACK_THRESHOLD = 40  # Both sensors below this = both on black
RECOVERY_STRAIGHT_TIME = 1.0  # Time to go straight during recovery (seconds)
SEARCH_SPEED = 30        # Speed for 360° search

# ===== SENSOR AND MOTOR INITIALIZATION =====
cl_L = ColorSensor('in4')  # Left sensor
cl_R = ColorSensor('in1')  # Right sensor
cl_L.mode = 'COL-REFLECT'
cl_R.mode = 'COL-REFLECT'

mL = LargeMotor(OUTPUT_D)  # Left motor
mR = LargeMotor(OUTPUT_A)  # Right motor

sound = Sound()
sound.beep()

# ===== CENTER-LINE FOLLOWER CLASS =====
class CenterLinePDFollower:
    def __init__(self, kp, kd, base_speed, max_speed, min_speed):
        self.kp = kp
        self.kd = kd
        self.base_speed = base_speed
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.last_error = 0
        
        # Recovery state variables
        self.recovery_state = "NORMAL"  # NORMAL, STRAIGHT_RECOVERY, SEARCH_360
        self.recovery_start_time = 0
        self.search_start_time = 0
        self.initial_search_direction = 1  # 1 for right, -1 for left
        
    def compute(self, left_value, right_value):
        """
        Center-line following with three-state recovery:
        1. Normal following (line between sensors)
        2. Straight recovery (go forward 1 second)
        3. 360° search (rotate until line found)
        """
        
        # Check current line status
        line_status = self._analyze_line_position(left_value, right_value)
        
        # State machine for recovery behavior
        if line_status == "LINE_LOST":
            if self.recovery_state == "NORMAL":
                # Start recovery: go straight first
                self.recovery_state = "STRAIGHT_RECOVERY"
                self.recovery_start_time = time.time()
                print("LINE LOST! Starting straight recovery...")
                return self._straight_recovery()
                
            elif self.recovery_state == "STRAIGHT_RECOVERY":
                # Continue straight recovery for 1 second
                if time.time() - self.recovery_start_time < RECOVERY_STRAIGHT_TIME:
                    return self._straight_recovery()
                else:
                    # Switch to 360° search
                    self.recovery_state = "SEARCH_360"
                    self.search_start_time = time.time()
                    print("Straight recovery failed. Starting 360° search...")
                    return self._start_360_search()
                    
            elif self.recovery_state == "SEARCH_360":
                # Continue 360° search until line found
                return self._continue_360_search()
        
        elif line_status == "LINE_FOUND":
            # Reset recovery state when line is found
            if self.recovery_state != "NORMAL":
                print("Line found! Resuming normal following.")
                sound.beep()
            self.recovery_state = "NORMAL"
            return self._normal_center_line_following(left_value, right_value)
        
        else:  # LINE_FOUND
            # Normal operation
            self.recovery_state = "NORMAL"
            return self._normal_center_line_following(left_value, right_value)
    
    def _analyze_line_position(self, left_value, right_value):
        """
        Determine if line is found or lost for center-line following
        """
        # For center-line following, we lose the line when both sensors
        # read very high values (both completely on white, away from line)
        if self._is_line_completely_lost(left_value, right_value):
            return "LINE_LOST"
        else:
            return "LINE_FOUND"
    
    def _is_line_completely_lost(self, left_value, right_value):
        """
        Check if line is completely lost (both sensors very high)
        This happens when robot goes completely off track
        """
        return (left_value > LINE_THRESHOLD + 20 and 
                right_value > LINE_THRESHOLD + 20)
    
    def _normal_center_line_following(self, left_value, right_value):
        """
        Center-line PD control:
        - Target: both sensors should read similar WHITE values
        - Error: difference between sensor readings
        - If left_value < right_value: robot drifted left, turn right
        - If right_value < left_value: robot drifted right, turn left
        """
        
        # Calculate error for center-line following
        error = left_value - right_value
        
        # Dynamic speed based on how centered we are
        both_values_avg = (left_value + right_value) / 2
        if both_values_avg > LINE_THRESHOLD + 15:  # Very white = very centered
            current_speed = self.max_speed
        elif both_values_avg > LINE_THRESHOLD:     # Somewhat centered
            current_speed = self.base_speed
        else:  # One or both sensors detecting line
            current_speed = self.min_speed
        
        # PD calculation
        P = self.kp * error
        D = self.kd * (error - self.last_error)
        self.last_error = error
        
        correction = P + D
        
        # Apply correction
        left_speed = current_speed - correction
        right_speed = current_speed + correction
        
        # Limit speeds
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        return left_speed, right_speed
    
    def _straight_recovery(self):
        """Go straight forward during recovery phase"""
        return self.base_speed, self.base_speed
    
    def _start_360_search(self):
        """Start 360-degree search rotation"""
        # Rotate in place - one motor forward, one backward
        return SEARCH_SPEED * self.initial_search_direction, -SEARCH_SPEED * self.initial_search_direction
    
    def _continue_360_search(self):
        """Continue 360-degree search until line is found"""
        # Keep rotating in the same direction
        return SEARCH_SPEED * self.initial_search_direction, -SEARCH_SPEED * self.initial_search_direction

# ===== KEYBOARD SETUP =====
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

# ===== MAIN PROGRAM =====
try:
    tty.setcbreak(sys.stdin.fileno())
    
    pd_controller = CenterLinePDFollower(KP, KD, BASE_SPEED, MAX_SPEED, MIN_SPEED)
    
    print("=== CENTER-LINE FOLLOWER WITH 360° RECOVERY ===")
    print("Line should stay BETWEEN the two sensors")
    print("Recovery: 1s straight → 360° search")
    print("Press 'q' to quit")
    print("Press 's' to start/stop")
    print("Press 'r' to reset controller")
    print("Press 'c' to calibrate sensors")
    print("===============================================")
    
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
                    print("STARTING center-line follower...")
                else:
                    sound.beep()
                    print("STOPPING...")
                    mL.on(SpeedPercent(0))
                    mR.on(SpeedPercent(0))
            elif c == 'r':
                pd_controller.last_error = 0
                pd_controller.recovery_state = "NORMAL"
                print("Controller reset")
            elif c == 'c':
                print("\n=== SENSOR CALIBRATION ===")
                print("Place robot so line is BETWEEN sensors...")
                time.sleep(3)
                centered_L = cl_L.value()
                centered_R = cl_R.value()
                print("Centered - Left: {}, Right: {}".format(centered_L, centered_R))
                
                print("Move robot OFF the line (both sensors on white)...")
                time.sleep(3)
                off_line_L = cl_L.value()
                off_line_R = cl_R.value()
                print("Off line - Left: {}, Right: {}".format(off_line_L, off_line_R))
                
                suggested_threshold = (min(centered_L, centered_R) + max(off_line_L, off_line_R)) / 2
                print("Suggested LINE_THRESHOLD: {:.0f}".format(suggested_threshold))
        
        if running and not stop_signal:
            left_value = cl_L.value()
            right_value = cl_R.value()
            
            left_speed, right_speed = pd_controller.compute(left_value, right_value)
            
            mL.on(SpeedPercent(left_speed))
            mR.on(SpeedPercent(right_speed))

            # Status display
            error = left_value - right_value
            avg_reading = (left_value + right_value) / 2
            
            status_info = "State: {}".format(pd_controller.recovery_state)
            if pd_controller.recovery_state == "STRAIGHT_RECOVERY":
                time_left = RECOVERY_STRAIGHT_TIME - (time.time() - pd_controller.recovery_start_time)
                status_info += " ({:.1f}s left)".format(time_left)
            
            print("L:{:2d} R:{:2d} | Avg:{:4.1f} | Err:{:3.0f} | ML:{:4.0f} MR:{:4.0f} | {}".format(
                left_value, right_value, avg_reading, error, left_speed, right_speed, status_info))
        
        elif not stop_signal:
            mL.on(SpeedPercent(0))
            mR.on(SpeedPercent(0))
        
        time.sleep(0.02)
    
    sound.beep()
    mL.on(SpeedPercent(0))
    mR.on(SpeedPercent(0))
    print("Program terminated")

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    mL.on(SpeedPercent(0))
    mR.on(SpeedPercent(0))