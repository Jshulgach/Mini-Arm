
import time
import board
from analogio import AnalogIn

newton_to_pounds = 0.2248089431 # Newtons to force-pounds

class ForceSensor:
    """ Object that handles force readings from a FSR. 
        
        Wiring consists of 3-5V going to one pin, the other split to analog input and GND, with a 10kOhm
        resistor before GND.
        
        Parameters:
        -----------
        pin        : (board) Pin assignment for analog inputs (between A1-A3)
        force_unit : (str)   Unit tyep for force reading conversion
    """
    def __init__(self, pin=board.A1, unit='newtons', max_val=65536, voltage=5):              
        self.pin = AnalogIn(pin)
        self.unit = unit
        self.max_val = max_val
        self.voltage = voltage
        
    @property
    def newtons(self):
        if self.unit == "newtons":
            return self.pin.value
        return   self.voltage/self.max_val * newton_to_pounds * self.value

    @property
    def pounds(self):
        if self.unit == "pounds":
            return self.pin.value
        return self.voltage/self.max_val * self.value / newton_to_pounds

