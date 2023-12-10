import time
import busio
import board
import usb_cdc

class USBSerialReader(object):
    """ Read a line from USB Serial (up to end_char), non-blocking, with optional echo 
    Depending on the microcontroller board used, the hardware may support multiple serial 
    communication methods. The RP2040 Pico has a "Console Direct Communication" (CDC)

    Parameters:
    -----------
    name              : (str)     The custom name for the class
    use_uart          : (bool)    The IP address to use to start the server
    port              : (int)     Port number on ip address to allow client connections
    rate              : (int)     Frequency of publishing rate for data to IP address
    simulate_hardware : (bool)    Allow physical control of hardware
    verbose           : (bool)    Enable/disable verbose output text to the terminal

    """
    def __init__(self, 
                 name="USBSerialReader",
                 use_UART=True, # If false, uses console
                 TERMINATOR='\r', 
                 COMMAND_DELIMITER=";",  # multiple commands should be parsed too
                 ARGUMENT_DELIMITER=":",
                 verbose=False,
                ):
        self.name = name
        self.use_UART = use_UART
        if self.use_UART:
            self.serial  = busio.UART(board.GP16, board.GP17, baudrate=9600, timeout=0)
        else:
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
        data = []
        
        # Decode the bytes currently stored in the buffer
        temp = self._out_data.decode("utf-8")
        
        # If a newline,return, or backspace character exists anywhere, those are not being parsed so we will remove them
        temp  = temp.replace('\n','').replace('\r','').replace('\b','')
        
        # Check if multiple commands were sent by splitting the message by the command delimiter
        cmds = temp.split(self.command_delimiter)
        
        for cmd in cmds:
            #data = cmd.split(self.argument_delimiter, 1)
            cmd_msg = cmd.split(self.argument_delimiter) # Want all possible arguments, not just the first

            if cmd_msg[-1] == '': del cmd_msg[-1] # Remove last element in arrray if empty            
            data.append(cmd_msg)
                    
        self._out_data = None
        return data
        
    def logger(self, *argv):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def update(self):
        """ When this function runs, it checks whether any bytes are waiting in the USB CDC buffer, and fills in 
            either the in_data or out_data buffer if the terminal byte is read.
        """
        if hasattr(self.serial, 'connected'):
            self.connected = self.serial.connected
                
        if self.serial.in_waiting > 0:
            if not self.connected: 
                self.connected = True
            
            while True:
                byte = self.serial.read(1)
                if byte:
                    if len(self.in_data) == 0 and byte == b'\r':
                        continue  # Skip adding the '\r' if it's the first character
                    if byte == self.terminator:
                        if self.verbose: 
                            self.logger("Terminator byte read. Sending {}".format(self.in_data))
                        self._out_data = self.in_data
                        self.in_data = bytearray()
                        break
                    else:
                        if byte.decode("utf-8") != '\n':
                            self.in_data += byte
                            if self.verbose: 
                                self.logger("New byte read. Buffer contents: {}".format(self.in_data.decode("utf-8")))
                else:
                    break
        
    def send(self, msg):
        """ Send a message to the connected serial device
        """
        self.serial.write(msg.encode("utf-8"))