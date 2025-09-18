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


# Actuators definition
mA = ev3.LargeMotor('outA')
mA.run_direct()

sound = Sound()
sound.beep()

# Non-blocking input setup
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())

    BASE_SPEED = 100
    stop_signal = 0

    while True:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':  # Quit when 'q' is pressed
                stop_signal = 1

        if stop_signal == 1:
            ev3.Sound.beep().wait()
            mA.duty_cycle_sp = 0

            break
        else:
            mA.duty_cycle_sp = BASE_SPEED

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)