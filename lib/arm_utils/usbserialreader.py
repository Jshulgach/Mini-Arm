import time
import usb_cdc

class USBSerialReader(object):
    """ Read a line from USB Serial (up to end_char), non-blocking, with optional echo """
    def __init__(self, 
                 TERMINATOR='\r', 
                 COMMAND_DELIMITER=";",  # multiple commands should be parsed too
                 ARGUMENT_DELIMITER=":",
                 verbose=False,
                ):
        self.name = "USBSerialReader"
        self.serial = usb_cdc.console
        self.in_data = bytearray()
        self._out_data = None
        self.terminator = TERMINATOR.encode('UTF8')
        self.command_delimiter = COMMAND_DELIMITER
        self.argument_delimiter = ARGUMENT_DELIMITER
        self.verbose = verbose
        self.connected = False
    
    @property
    def out_data(self):
        temp = self._out_data.decode("utf-8")
        #data = temp.split(self.argument_delimiter, 1)
        data = temp.split(self.argument_delimiter) # Want all possible arguments, not just the first
        if data[-1] == '': del data[-1]
        self._out_data = None
        return data
        
    def logger(self, *argv):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def update(self):
        """ When this function runs, it checks whether any bytes are waiting in the USB CDC buffer, and fills in 
            either the in_data or out_data buffer if the terminal byte is read.
        """
        self.connected = self.serial.connected
        if self.serial.in_waiting > 0:
            byte = self.serial.read(1)
            if byte == self.terminator:
                if self.verbose: self.logger("Terminator byte read")
                self._out_data = self.in_data
                self.in_data = bytearray()
            else:
                self.in_data += byte
                if self.verbose: self.logger("New serial byte read. Buffer contents: {}".format(self.in_data.decode("utf-8")))

        