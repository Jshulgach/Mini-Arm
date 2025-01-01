import pwmio
import digitalio
import board

class RGBLED:
    def __init__(self, rpin=board.GP3, gpin=board.GP4, bpin=board.GP5, freq=1000, brightness=0.5, set_color=None):      
        self.brightness = brightness
        self.led = {'Red':   pwmio.PWMOut(rpin, frequency=freq, duty_cycle=self.convert(0)),
                    'Green': pwmio.PWMOut(gpin, frequency=freq, duty_cycle=self.convert(0)),
                    'Blue':  pwmio.PWMOut(bpin, frequency=freq, duty_cycle=self.convert(0))
                    }
        # RGB LED configuration, using pin2 as the 3.3V output source, and 3-5 as input GND pins
        self.rgb_out = digitalio.DigitalInOut(board.GP2)
        self.rgb_out.direction = digitalio.Direction.OUTPUT
        self.rgb_out.value = True
        self.all_off()
        if set_color:
            self.set_color(set_color)

    def convert(self, value, cathode=True):
        leftSpan = 100 # 0-100
        rightSpan = 65025 # 0-65025

        # adjust brightness of LED ( scale from 0 (off) to 1 (full power)
        duty_cycle = int(rightSpan*self.brightness)

        valueScaled = float(value) / leftSpan # Convert the left range into a 0-1 range (float)        
        duty_cycle = valueScaled * rightSpan # Convert the 0-1 range into a value in the right range.
        if cathode:
            # f using the cathode RGB led make sure to flip the value
            duty_cycle = rightSpan - duty_cycle       
        return int(duty_cycle)

    def all_off(self):
        self.set_brightness('Red', 0)
        self.set_brightness('Green', 0)
        self.set_brightness('Blue', 0)

    def all_on(self):
        self.set_brightness('Red', 100)
        self.set_brightness('Green', 100)
        self.set_brightness('Blue', 100)
        
    def set_brightness(self, pin, val):
        ''' Helper function that controls the LED brightness from 0-100%
        '''
        if pin in self.led:
            self.led[pin].duty_cycle = self.convert(val)
        
    def set_color(self, values):
        """ Function that changes the color of the LED

        :param values: (list) A list of 3 elements ranging from 0-100 for R,G,B colors
        """
        self.led['Red'].duty_cycle = self.convert(values[0])
        self.led['Green'].duty_cycle = self.convert(values[1])
        self.led['Blue'].duty_cycle = self.convert(values[2])