# SPDX-FileCopyrightText: 2021 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""Example for Pico. Blinks the built-in LED."""
import time
import board
import digitalio
import busio

TIME_LIMIT = 2;
setTimer = 0;

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

uart = busio.UART(board.GP16, board.GP17, baudrate=9600, timeout=0)

led.value = True
time.sleep(0.1)
led.value = False
time.sleep(0.1)
led.value = True
time.sleep(0.1)  
led.value = False


while True:
    #led.value = True
    #time.sleep(0.5)
    #led.value = False
    #time.sleep(0.5)
    if uart.in_waiting > 0:
        data = uart.read(1)  # read up to 32 bytes
        led.value = True
        time.sleep(0.1)
        led.value = False
        time.sleep(0.1)



    if (time.time() - setTimer >= TIME_LIMIT):
        led.value = False
        time.sleep(0.1)
        led.value = True
        time.sleep(0.1)
        led.value = False
        time.sleep(0.1)
        uart.write('hello'.encode('utf-8'))
        setTimer = time.time()
        
    
        #if data is not None:
        #    led.value = True
        #    time.sleep(0.3)
        #    led.value = False
        
    
    