import threading
from threading import Timer, Thread
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA
from machine import Pin, PWM

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

# Server page
server_page = """<!DOCTYPE html>
<html>
    <head> <title>Pico W</title> </head>
    <body> <h1>Pico W</h1>
        <p>%s</p>
    </body>
</html>
"""


class RepeatTimer(Timer):
    """
    Timer object inherited from the threading package, modified for repeating execution
    """

    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


class PCAServo(servo, pca):
    def __init__(self, name=None, ch=1, min_pulse=400, max_pulse=2400):
        """Servo class that uses the PCA9685 instance to configure servos on the pca board
        To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
        match the stall points of the servo.

        :param name: (str) unique servo name
        :param ch: (int) channel to assign on the pca board
        :param min_pulse: (int) pulse width modulation lower bound, servo specific
        :param max_pulse: (int) pulse width modulation upper bound
        """
        self.name = name
        self.ch = ch
        self._servo = servo.Servo(pca.channels[ch], min_pulse=min_pulse, max_pulse=max_pulse)

    def write(self, val):
        self._servo.angle(val)


class RGBLED(Pin, PWM):
    def __init__(self, rpin, gpin, bpin, freq=1000):
        self.rpin = rpin
        self.gpin = gpin
        self.bpin = bpin
        self.freq = freq
        self.pwm = {'Red': PWM(Pin(self.rpin), freq=self.freq),
                    'Green': PWM(Pin(self.gpin), freq=self.freq),
                    'Blue': PWM(Pin(self.bpin), freq=self.freq)
                    }
        self.all_off()

    def all_off(self):
        self.set_brightness('Red', 0)
        self.set_brightness('Green', 0)
        self.set_brightness('Blue', 0)

    def change_color(self, color):
        """ Function that changes the color of the LED

        :param color: (list) A list of 3 elements ranging from 0-100 for R,G,B colors
        """
        self.set_brightness('Red', color[0])
        self.set_brightness('Green', color[1])
        self.set_brightness('Blue', color[2])

    def set_brightness(self, pin, val):
        self._set_duty(pin, self.translate(val))

    def _get_duty(self, pin):
        return self.pwm[pin].duty_u16()

    def get_freq(self, pin):
        return self.pwm[pin].freq()

    def _set_duty(self, pin, val):
        self.pwm[pin].duty_u16(val)

    def set_freq(self, pin, val):
        self.pwm[pin].freq(val)

    def translate(self, value):
        # Figure out how 'wide' each range is
        leftSpan = 100
        rightSpan = 65025

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value) / leftSpan

        # Convert the 0-1 range into a value in the right range.
        return valueScaled * rightSpan

