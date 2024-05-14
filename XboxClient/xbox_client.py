import socket
import sys
import time
import asyncio
import serial
from threading import Timer
from xbox_control import XboxController, XBOX_MAP
import numpy as np

__version__ = "1.0.1"

MESSAGE_TERMINATOR = ";"
MAXSTEP = 51
TOLERANCE = 0.1

def interpolation(d, x):
    output = d[0][1] + (x - d[0][0]) * ((d[1][1] - d[0][1])/(d[1][0] - d[0][0]))
 
    return output
 

def create_circular_trajectory(center, radius=10, steps=101):
    theta = np.linspace(0, 2 * np.pi, steps)
    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
    return center + radius * temp


points = create_circular_trajectory([130, 0, 270], 40, MAXSTEP)  # draw a circle
euler = np.array([0, 0, 0])  # keep same orientation for all points


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False


class XboxClient:
    def __init__(self,
                 name='XboxClient',
                 ip='192.168.1.165',  # for the dedicated dns once it goes back up: 'desktoparm.ddns.net'
                 serialport='COM5',
                 baudrate=115200,
                 rate=2,
                 simulate=False,
                 online=False,
                 use_serial=True,
                 verbose=True,
                 ):
        """
        ControllerClient is an object that acts as a simple TCP Client and transmits the state of a connected
        controller for tele-operation.

        Parameters:
        -----------
        node_name         : (str)     A specific node name
        ip                : (str)     The IP address to use to start the server
        serialport        : (int)     Port number on ip address to allow client connections
        baudrate          : (int)     Baudrate for serial communication
        rate              : (int)     Frequency of publishing rate for client
        simulate          : (bool)    Allow physical control of hardware
        online            : (bool)    Enable/Disable server
        use_serial        : (bool)    Enable/Disable serial command parsing
        verbose           : (bool)    Enable/disable verbose output text to the terminal

        """
        # if len(sys.argv) > 1:
        #    self.ip = str(sys.argv[1])
        # self.serialport = serialport
        # if len(sys.argv) > 2:
        #    self.serialport = int(sys.argv[2])

        self.name = name
        self.ip = ip
        self.rate = rate
        self.simulate = simulate
        self.online = online
        self.use_serial = use_serial
        self.verbose = verbose

        self.connected = False
        self.counter = 0
        self.usb_serial = None
        self.controller_exists = False
        self.last_gripper_command = None
        self.all_stop = False
        self.prev_data = []
        self.prev_msg = ""
        self.timer = None

        # Create controller object with button event handling
        self.xbox = XboxController()

        if self.use_serial:
            self.usb_serial = self.connect_to_serial(serialport, baudrate)

        if self.verbose:
            self.logger("{} object created!".format(self.name))

        # self.controller_exists = True if len(self.xbox.gamepads) > 0 else print(
        #    "Joystick controller created. Connect a joystick controller to begin transmitting...")

        # Attempt to connect to server
        # if self.verbose: print('Attempting to connect to {} port {}'.format(self.ip, 1000)) # default port 1000
        # self.connect(self.ip, 1000)

    def logger(self, *argv):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def start(self):
        """ Makes a call to the asyncronous library to run a main routine """
        asyncio.run(self.main())  # Need to pass the async function into the run method to start
        # self.timer = RepeatedTimer(1 / self.rate, function=self.main)

    def stop(self):
        """ Sets a flag to stop running all tasks """
        self.all_stop = True

    async def main(self):
        """ Start main tasks and coroutines in a single main function
        """

        # if self.online:
        #    """ Note: Enabling this asyncronous task will allow commands from the network to be received, but large
        #        strings will bog down the queueing message handler.
        #    """
        #    self.logger(
        #        "Setting up webserver on ( ip: '{}', Port: {} ) ...".format(str(wifi.radio.ipv4_address), self.port))
        #    asyncio.create_task(asyncio.start_server(self.serve_client, str(wifi.radio.ipv4_address), self.port))

        # Helper routine to keep publishing messages to the queue instead of waiting on a client, disable after debug
        # asyncio.create_task(self.helper_queue_msgs(50))  # 50

        self.logger("Setting up robot update rate")
        asyncio.create_task(self.update(10))

        self.logger("{} running!".format(self.name))

        while not self.all_stop:
            await asyncio.sleep(0)  # Calling #async with sleep for 0 seconds allows coroutines to run


    async def update(self, interval=100): 
        """ A callback function that gets called every "1/rate" seconds
        The function reads the current state of the joystick and sends it as a string to the
        connected ip address and port

        """
        i = 0
        while not self.all_stop:
            if self.controller_connected() and self.usb_serial:

                cmd = self.check_gripper()
                if cmd:
                    self.logger("Sending: {}".format(str(cmd)))
                    self.usb_serial.write(cmd.encode('utf8'))
                cmd = self.check_home_button()
                if cmd:
                    self.logger("Sending: {}".format(str(cmd)))
                    self.usb_serial.write(cmd.encode('utf8'))

                # Choosing serial command structure here. Sending commands to the robot server is disabled for now
                cmd = self.joy_to_delta()
                if cmd:
                    self.logger("Sending: {}".format(str(cmd)))
                    self.usb_serial.write(cmd.encode('utf8'))
                    
                #cmd = "robotinfo;"
                #cmd = "getpose;"
                # Get FSR Readings
                #cmd = "fsr;"                
                #self.usb_serial.write(cmd.encode('utf8'))
                if i > 50:
                    self.usb_serial.write("info;".encode('utf8'))
                    i=0
                i = i + 1
                    
                if self.usb_serial.inWaiting():
                    print("[{:.3f}][Robot] {}".format(time.monotonic(), self.usb_serial.readline().decode('utf8')), end="")

            await asyncio.sleep(1 / int(interval))
            # await asyncio.sleep(0) # handle this as fast as possible compared to the other coroutines

    def check_home_button(self):
        """ Function that determines whether to set the robot to its home position

        Home == START, BTN_START
        
        """
        cmd = None
        data = self.xbox.read()
        if data[XBOX_MAP['Start']] > 0:
            cmd = "ikhome" + MESSAGE_TERMINATOR
        return cmd

    def check_gripper(self):
        """ Function that determines whether to open or close the gripper
        
        Gripper angle == RightTrigger
        Gripper Open  == A, BTN_SOUTH
        Gripper close == B, BTN_EAST

        """
        dg = 180
        data = self.xbox.read()
        if abs(data[XBOX_MAP['RightTrigger']]) > 0:
            dg = interpolation([[0, 180],[1, 0]], data[XBOX_MAP['RightTrigger']])
        #if data[XBOX_MAP['A']] > 0:
        #    dg = 180
        if data[XBOX_MAP['B']] > 0:
            dg = 0
        
        cmd = "gripper:" + str(dg) + MESSAGE_TERMINATOR
        if (cmd == self.last_gripper_command):
            return None
            
        self.last_gripper_command = cmd
        return cmd

    def joy_to_delta(self):
        """ Function that converts the keymapping from the gamepad to delta commands for the robot
        Key mapping is similar to Piknik Robotics controller mapping for their Panda robot arm. Uses the updating rate
        to determine step inputs like a velocity controller

        Z-direction down      == LeftButton
        Z-direction up        == RightButton
        Y-direction motion    == RightJoystickX
        X-direction motion    == RightJoystickY
        X-axis rotation       == LeftJoystickX
        Y-axis rotation       == LeftJoystickY
        Z-axis rotation (neg) == LeftBumper
        Z-axis rotation (pos) == RightBumper

        """
        cmd = None
        data = self.xbox.read()
        dx = 0.00
        dy = 0.00
        dz = 0.00
        gain = 0.005
        if abs(data[XBOX_MAP['RightJoystickY']]) > TOLERANCE:
            dx = gain * data[XBOX_MAP['RightJoystickY']]
        if abs(data[XBOX_MAP['RightJoystickX']]) > TOLERANCE:
            dy = gain * data[XBOX_MAP['RightJoystickX']]
        if abs(data[XBOX_MAP['RightBumper']]) > TOLERANCE:
            dz = gain * float(data[XBOX_MAP['RightBumper']])
        if abs(data[XBOX_MAP['LeftBumper']]) > TOLERANCE:
            dz = -gain * float(data[XBOX_MAP['LeftBumper']])
       
        if any((dx, dy, dz)):       
            cmd = "delta:0.0,0.0," + str(dz) + MESSAGE_TERMINATOR
            #cmd = "delta:0.0,0.0," + str(dz) + ",0,0,0" + MESSAGE_TERMINATOR
            #cmd = "delta:" + str(dx) + "," + str(dy) + "," + str(dz) + ",0,0,0" + MESSAGE_TERMINATOR

        return cmd

    def connect(self, ip, port):
        """ Function that makes an attempt to connect the socket to the port where the server
        is listening. If successful, it starts the timer object to send data to IP address at specified rate

        Parameters:
        -----------
        ip   : (str)     The IP address that the server is on
        port : (int)     Port number specific to server

        """
        while not self.connected:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a TCP/IP socket.
                self.sock.connect((ip, port))
                self.connected = True
                self.usb_serial.write("robotinfo;".encode('utf8'))
                self.usb_serial.write("led:0,100,0;".encode('utf8')) # Changes it to green
                if self.verbose: print('Connection to server at {} successful'.format((ip, port)))
                self.timer.start()

            except KeyboardInterrupt:
                if self.verbose: print("KeyboardInterrupt detected")
                self.connected = True
                self.stop()

            except socket.error:
                self.connected = False
                self.timer.stop()

    def controller_connected(self, wait=True):
        """ Helper function that check whether a gamepad controller device is connected to the computer. Option to
        wait for connection

        Parameters:
        -----------
        wait     : (bool) Enable.disable waiting for connection before exiting function

        Return:
        ----------
        (bool) True if connected, otherwise false

        """
        if len(self.xbox.gamepads) == 0:
            self.controller_exists = False
            self.logger("Warning - No controller detected. Please plugin a controller to start teleoperation")
            if not wait:
                return False
            else:
                while len(self.xbox.gamepads) < 1:
                    time.sleep(1)
                self.logger("New controller detected!")
        self.controller_exists = True
        return True

    def disconnect(self):
        """ Function to explicitly disconnect from microcontroller if connected and clean serial use 
        to allow new connection 
        """
        if self.sock:
            self.sock.close()
            if self.verbose: print('Socket closed')

    def stop(self):
        """ Explicit command to stop the client """
        #self.timer.stop()
        #self.disconnect()
        pass

    def connect_to_serial(self, port, baud):
        s = None
        if self.use_serial:
            self.logger("Serial Enabled. Looking for device on port {}".format(port))
            try:
                s = serial.Serial(port=port, baudrate=baud, timeout=0.0)
                s.write("led:0,100,0;".encode('utf8')) # Changes it to green
                if s.is_open: self.logger("Device found. Successful connection using port {}".format(s.port))
            except:
                self.logger("Failure to connect to device on ")
        return s
        
    def TODO_sock_send(self):
        """
        """
        try:
            pass

        except KeyboardInterrupt:
            if self.verbose: print("KeyboardInterrupt detected")
            self.connected = False
            self.stop()

        except socket.error:
            if self.verbose: print('Connection to the server lost, attempting to reconnect...')
            self.connected = False
            self.timer.stop()

    def TODO_joy_to_controller(self):
       """ TO-DO convert joystick messages to CONTROLLER command type for robot
       """
       joy_data = []
       joystick_indices = [0, 1, 2, 3, 4, 5]
       data = self.xbox.read()
       if not all(abs(i) < TOLERANCE for i in data):
           for j in data:
               if abs(j) >= TOLERANCE:
                   joy_data.append(j)
               else:
                   joy_data.append(0.0)
               # joy_data = [data[i] for i in joystick_indices]

               msg = ",".join([str(elem) for elem in joy_data])
               msg = "controller " + msg + MESSAGE_TERMINATOR  # adding the keyword for the controller command parser, and terminator
               # msg =  "pose " + ",".join([str(i) for i in points[self.counter]]) +",0,0,0"+ MESSAGE_TERMINATOR
               # if all(data[i]>0 for i in joystick_indices)

               # self.sock.send(msg.encode())
               # self.prev_msg = msg
               # self.prev_joy = joy_data

               # if self.verbose:
               #    self.logger('[{}] Sent message: {!r}'.format(self.counter, msg))

               # if self.counter >= MAXSTEP - 1:
               #    self.counter = 0
               # else:
               #    self.counter += 1


