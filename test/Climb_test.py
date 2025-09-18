#!/usr/bin/env python3
'''
SDU - Emboided AI Project : Speedy Gonzales
-   Jose Angel Huerta Rios
-   Andres De Pool
-   Panagiotis Nikolaodis

'''
import sys
import signal
import argparse
from time import sleep
import ev3dev.ev3 as ev3
import select
import tty
import termios

from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveTank, SpeedPercent, LineFollowErrorLostLine, LineFollowErrorTooFast, follow_for_ms, follow_for_forever

# Sensors definition
cl = ev3.ColorSensor('in4')
cl.mode = 'COL-REFLECT'
assert cl.connected, "Color sensor is not connected"

# Actuators definition
mR = ev3.LargeMotor('outA')
mL = ev3.LargeMotor('outD')
mR.run_direct()
mL.run_direct()

sound = Sound()
sound.beep()

# Non-blocking input setup
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())

    BASE_SPEED = -90
    TURN_SPEED = 80
    stop_signal = 0

    while True:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':  # Quit when 'q' is pressed
                stop_signal = 1

        if stop_signal == 1:
            ev3.Sound.beep().wait()
            mR.duty_cycle_sp = 0
            mL.duty_cycle_sp = 0
            break
        else:
            mR.duty_cycle_sp = BASE_SPEED
            mL.duty_cycle_sp = BASE_SPEED
            print(str(cl.value()))

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)