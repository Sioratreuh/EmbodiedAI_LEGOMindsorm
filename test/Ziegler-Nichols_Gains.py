#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Herramienta para encontrar las ganancias KP y KD usando el método de Ziegler-Nichols
- Solo usa control proporcional (PD con KD=0)
- Permite aumentar/disminuir KP desde la terminal
- Muestra el valor actual de KP y el comportamiento del robot
- Cuando observes oscilación sostenida, anota KP (Kcr) y el periodo de oscilación (Pcr)
- Calcula y muestra las ganancias sugeridas para PD según Ziegler-Nichols
"""

import sys
import select
import tty
import termios
import time

from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_B, SpeedPercent

# ===== CONFIGURACIÓN INICIAL =====
KP = 5.0
KD = 0.0  # SIEMPRE EN CERO PARA EL MÉTODO
BASE_SPEED = 28
MAX_SPEED = 80
MIN_SPEED = 10
LINE_THRESHOLD = 10
SAMPLE_INTERVAL = 0.02
INVERT_MOTORS = False

cl_L = ColorSensor('in4')
cl_R = ColorSensor('in1')
try:
    cl_L.mode = 'COL-REFLECT'
    cl_R.mode = 'COL-REFLECT'
except Exception:
    pass

mL = LargeMotor(OUTPUT_D)
mR = LargeMotor(OUTPUT_A)
mB = LargeMotor(OUTPUT_B)

sound = Sound()
sound.beep()

def apply_motor_speeds(left_speed, right_speed, rear_speed, invert=INVERT_MOTORS):
    l = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    r = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    b = max(min(rear_speed, MAX_SPEED), -MAX_SPEED)
    mL.on(SpeedPercent(l))
    mR.on(SpeedPercent(r))
    mB.on(SpeedPercent(b))
    return l, r, b

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())
    print("=== ZIEGLER-NICHOLS PD TUNER ===")
    print("Usa '+' y '-' para ajustar KP. 's' start/stop, 'q' quit")
    print("Cuando observes oscilación sostenida, anota KP (Kcr) y el periodo (Pcr)")
    print("Luego usa la fórmula: KP=0.8*Kcr, KD=0.12*Kcr*Pcr")
    running = False
    stop_signal = False
    kp = KP

    while not stop_signal:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':
                stop_signal = True
                sound.beep()
            elif c == 's':
                running = not running
                if running:
                    sound.beep()
                    print("STARTING...")
                else:
                    print("STOPPING...")
                    apply_motor_speeds(0.0, 0.0, 0.0)
            elif c == '+':
                kp += 1.0
                print("KP aumentado a: {:.2f}".format(kp))
            elif c == '-':
                kp = max(0.0, kp - 1.0)
                print("KP disminuido a: {:.2f}".format(kp))

        if running and not stop_signal:
            try:
                left_value = cl_L.value()
                right_value = cl_R.value()
            except Exception:
                left_value = cl_L.reflected_light_intensity
                right_value = cl_R.reflected_light_intensity

            error = float(right_value) - float(left_value)
            correction = kp * error
            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction
            rear_speed = BASE_SPEED
            applied_l, applied_r, applied_b = apply_motor_speeds(left_speed, right_speed, rear_speed)
            avg = (left_value + right_value) / 2.0
            print("KP:{:5.2f} | L:{:3d} R:{:3d} | Avg:{:4.1f} | Err:{:6.2f} | ML:{:5.1f} MR:{:5.1f} MB:{:5.1f}".format(
                kp, int(left_value), int(right_value), avg, error, applied_l, applied_r, applied_b))
        elif not stop_signal:
            apply_motor_speeds(0.0, 0.0, 0.0)

        time.sleep(SAMPLE_INTERVAL)

    apply_motor_speeds(0.0, 0.0, 0.0)
    print("Programa terminado")
    print("Cuando observes oscilación sostenida, anota KP (Kcr) y el periodo (Pcr)")
    print("Luego usa la fórmula: KP=0.8*Kcr, KD=0.12*Kcr*Pcr")

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    apply_motor_speeds(0.0, 0.0, 0.0)