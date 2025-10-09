import ev3dev.ev3 as ev3

from time import sleep

import signal

btn = ev3.Button()

mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outB')

THRESHOLD_LEFT = 30
THRESHOLD_RIGHT = 350

BASE_SPEED = 20
TURN_SPEED = 80

TouchSensor = ev3.TouchSensor('in2')
us = ev3.UltrasonicSensor('in1')
cl = ev3.ColorSensor('in3')
gy = ev3.GyroSensor('in4')

us.mode = 'US-DIST-CM'
cl.mode = 'COL-COLOR'
gy.mode = 'GYRO-ANG'

mB.run_direct()
mA.run_direct()

assert us.connected, "Ultrasonic sensor is not connected"
assert TouchSensor.connected, "Touch sensor is not connected"
assert cl.connected, "Color sensor is not connected"
assert gy.connected, "Gyro sensor is not connected"

colors = ('unknown','black','blue','green','yellow','red','white','brown')

while True:
    mA.duty_cycle_sp= BASE_SPEED
    mB.duty_cycle_sp= BASE_SPEED
    tou_val = TouchSensor.value()

    if tou_val == 1:
        ev3.Sound.beep().wait()
        mA.duty_cycle_sp= 0
        mB.duty_cycle_sp= 0
        exit()
    else:
        print(str(gy.value()))

    