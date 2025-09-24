#!/usr/bin/env python3
'''
SDU - Embedded AI Project: Speedy Gonzales
- Jose Angel Huerta Rios
- Andres De Pool
- Panagiotis Nikolaodis

PD Controller Line Follower
'''
import sys
import select
import tty
import termios
import time
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent

# ===== PD CONFIGURATION =====
KP = 0.8    # Proportional gain (adjust based on testing)
KD = 1.2    # Derivative gain (adjust based on testing)
BASE_SPEED = -10  # Base motor speed
TARGET = 25      # Target value for line following (adjust based on sensor readings)

# ===== SENSOR AND MOTOR INITIALIZATION =====
# Color sensors
cl_L = ColorSensor('in4')
cl_R = ColorSensor('in1')
cl_L.mode = 'COL-REFLECT'
cl_R.mode = 'COL-REFLECT'

# assert cl_L.conn, "Left sensor not connected"
# assert cl_R.connected, "Right sensor not connected"

# Motors
mR = LargeMotor(OUTPUT_A)  # Right motor
mL = LargeMotor(OUTPUT_D)  # Left motor
mB = LargeMotor(OUTPUT_B)  # Additional motor (if needed)

# assert mR.connected, "Right motor not connected"
# assert mL.connected, "Left motor not connected"

# Sound
sound = Sound()
sound.beep()

# ===== PD CONTROLLER =====
class PDFollower:
    def __init__(self, kp, kd, target, base_speed):
        self.kp = kp
        self.kd = kd
        self.target = target
        self.base_speed = base_speed
        self.last_error = 0
    
    def compute(self, left_value, right_value):
        # Calculate error: difference between sensors
        error = left_value - right_value
        
        # Proportional term
        P = self.kp * error
        
        # Derivative term (change in error)
        D = self.kd * (error - self.last_error)
        self.last_error = error
        
        # Total correction
        correction = P + D
        
        # Apply correction to motors
        left_speed = self.base_speed - correction
        right_speed = self.base_speed + correction
        
        # Limit speeds between -100 and 100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        return left_speed, right_speed

# ===== KEYBOARD SETUP =====
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

# ===== MAIN PROGRAM =====
try:
    tty.setcbreak(sys.stdin.fileno())
    
    # Create PD controller
    pd_controller = PDFollower(KP, KD, TARGET, BASE_SPEED)
    
    print("=== PD LINE FOLLOWER ===")
    print("Press 'q' to quit")
    print("Press 's' to start/stop")
    print("Press 'r' to reset PD controller")
    print("========================")
    
    running = False
    stop_signal = False
    
    while not stop_signal:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':  # Quit
                stop_signal = True
            elif c == 's':  # Start/Stop
                running = not running
                if running:
                    sound.beep()
                    print("STARTING line follower...")
                else:
                    sound.beep()
                    print("STOPPING...")
                    mL.on(SpeedPercent(0))
                    mR.on(SpeedPercent(0))
                    mB.on(SpeedPercent(0))
            elif c == 'r':  # Reset PD controller
                pd_controller.last_error = 0
                print("PD controller reset")
        
        if running and not stop_signal:
            # Read sensors
            left_value = cl_L.value()
            right_value = cl_R.value()
            
            # Calculate motor speeds with PD controller
            left_speed, right_speed = pd_controller.compute(left_value, right_value)
            
            # Apply speeds to motors
            mL.on(SpeedPercent(left_speed))
            mR.on(SpeedPercent(right_speed))
            mB.on(SpeedPercent(-min(left_speed,right_speed)))
            # Display debug information
            print("L:" + str(left_value) + "| R: "+ str(right_value)+" | ML: "+str(left_speed)+" | MR: "+ str(right_speed))
        
        elif not stop_signal:
            # Not running, keep motors off
            mL.on(SpeedPercent(0))
            mR.on(SpeedPercent(0))
            mB.on(SpeedPercent(0))
        
        # Small delay to avoid overloading
        time.sleep(0.02)
    
    # Cleanup on exit
    sound.beep()
    mL.on(SpeedPercent(0))
    mR.on(SpeedPercent(0))
    mB.on(SpeedPercent(0))

    print("Program terminated")

finally:
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    mL.on(SpeedPercent(0))
    mR.on(SpeedPercent(0))