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
                 baudrate=9600,
                 TERMINATOR='\r', 
                 COMMAND_DELIMITER=";",  # multiple commands should be parsed too
                 ARGUMENT_DELIMITER=":",
                 verbose=False,
                ):
        self.name = name
        self.use_UART = use_UART
        if self.use_UART:
            self.serial  = busio.UART(board.GP16, board.GP17, baudrate=baudrate, timeout=0)
        else:
            self.serial = usb_cdc.console

        self.in_data = bytearray()
        self._out_data = None
        self.terminator = TERMINATOR.encode('UTF8')
        self.command_delimiter = COMMAND_DELIMITER
        self.argument_delimiter = ARGUMENT_DELIMITER
        self.verbose = verbose
        self.connected = False
        self._pending = b''
        self._discard = False
        if len(self.terminator) != 1:
            raise ValueError("Use a single-byte command terminator")
    
    @property
    def out_data(self):
        raw = self._out_data
        self._out_data = None
        if not raw:
            return []
        try:
            text = raw.decode('ascii')
        except UnicodeError:
            self.logger('ERROR invalid command encoding')
            return []
        return [part.split(self.argument_delimiter) for part in text.split(self.command_delimiter) if part]

    def logger(self, *argv):
        print('[{:.3f}][{}] {}'.format(time.monotonic(), self.name, ''.join(argv)))

    def update(self):
        """Read a bounded chunk; deliver one complete frame per scheduler turn.

        Oversized frames are discarded through their terminator, never executed
        as a truncated command. Bytes after a delimiter remain for the next turn.
        """
        self.connected = getattr(self.serial, 'connected', True)
        if self._out_data is not None:
            return
        if not self._pending and self.serial.in_waiting:
            self._pending = self.serial.read(min(self.serial.in_waiting, 256)) or b''
        if not self._pending:
            return
        index = self._pending.find(self.terminator)
        chunk = self._pending if index < 0 else self._pending[:index]
        self._pending = b'' if index < 0 else self._pending[index + 1:]
        chunk = chunk.replace(b'\r', b'').replace(b'\n', b'')
        if not self._discard:
            if len(self.in_data) + len(chunk) > 512:
                self.in_data = bytearray()
                self._discard = True
                self.logger('ERROR command exceeds 512 bytes')
            else:
                self.in_data.extend(chunk)
        if index >= 0:
            if not self._discard and self.in_data:
                self._out_data = self.in_data
            self.in_data = bytearray()
            self._discard = False

    def send(self, msg):
        self.serial.write(msg.encode('utf-8'))
