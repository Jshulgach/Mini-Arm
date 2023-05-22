# This is a test of the servo capabilities from the Adafruit ServoKit and being able to update the servos at
# a quick enough rate so that the jerkiness can be removed for the PICO 

#
# Recorded an update rate of 120Hz 

import time
import busio
import board
import ulab.numpy as np
from adafruit_servokit import ServoKit

servos = ServoKit(channels=16, i2c=busio.I2C(board.GP1, board.GP0))

# Make range of values from 0-to-2pi
vals = np.linspace(0, 2*np.pi, 201)
prev_t = 0

while True:
    for i in vals:
        servos.servo[11].angle = round(90 + 90*np.cos(i))
        servos.servo[12].angle = round(90 + 90*np.sin(i))
        print("Motor 1: {}  |  Motor 2: {} | Rate {}Hz".format(round(90+90*np.cos(i)), round(90+90*np.sin(i)), 1/(time.monotonic()-prev_t)))
        prev_t = time.monotonic()
        #time.sleep(0.05)
