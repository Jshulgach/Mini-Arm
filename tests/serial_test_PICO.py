import sys
import time
from arm_utils.usbserialreader import USBSerialReader
from rgbled import RGBLED

rgb = RGBLED(set_color=[30, 0, 0])
serial = USBSerialReader(verbose=True)

while True:
    serial.update()
    if serial.in_data:
        rgb.set_color([100, 100, 100])
    
    if serial._out_data:
        data = serial.out_data[0]
        # output message using print function
        print("print: {}\n".format(data))
        if data=='b':
            #serial.out_data = None
            rgb.set_color([0, 0, 100])
        if data=='g':
            rgb.set_color([0, 100, 0])
        if data==';':
            rgb.set_color([100, 0, 0])

        # Write message to host whrough the system. Both output methods should work
        msg = 'stdout: ' + str(data) + '\n'
        sys.stdout.write(msg.encode("utf-8"))

    time.sleep(0.01) 
