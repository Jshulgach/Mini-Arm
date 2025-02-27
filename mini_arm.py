import time
import serial
import threading
import numpy as np

__version__ = '0.1.0'
__author__ = 'Jonathan Shulgach'


class MiniArmClient(object):
    """ A class that serves as a client for the MiniArm robot arm and acts like a driver

    Parameters
    ----------
    name (str) : Name of the client
    port (str) : Serial port to connect to
    baudrate (int) : Baudrate for the serial connection
    visualize (bool) : Enable/disable visualization
    verbose (bool) : Enable/disable verbose output
    """

    def __init__(self, name='MiniArmClient', port='COM3', baudrate=9600, command_delimiter=';', verbose=False):
        self.name = name
        self.command_delimiter = command_delimiter
        self.verbose = verbose

        # Setup serial communication
        self.s = serial.Serial(port, baudrate, timeout=1)
        self.connected = True if self.s.is_open else False

        # Print out any bytes currently in the buffer
        data = self.get_buffer()

        # Set the debug mode to match verbose
        #self.send_message(f"debug:{self.verbose}")

        self.home() # Set to home position upon successful connection

        # Move to a new pose
        self.send_message('set_pose:[0.135, 0.000, 0.22]')
        #self.send_message('set_delta_pose:[0.0,0.0,-0.02]')

        self.logger(f"{self.name} initialized\n")

    def logger(self, *argv, warning=False):
        """ Robust logging function to track events"""
        msg = ''.join(argv)
        if warning: msg = '(Warning) ' + msg
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def send_message(self, message):
        """Sends a message to the Pico over serial. Add the command terminator if missing.

        Parameters:
        -----------
        message    (str): The message to send.
        """
        if self.s and self.s.is_open:
            if not message.endswith(self.command_delimiter):
                message += self.command_delimiter  # Add terminator character to the message

            try:
                self.s.write(message.encode())
                if self.verbose:
                    print(f"Sent message to Pico: {message}")

                time.sleep(0.01) # Mandatory 10ms delay to prevent buffer overflow

            except serial.SerialException as e:
                print(f"Error sending message: {e}")
        else:
            print("Serial connection not available or not open.")

    def get_buffer(self):
        """Read all available bytes from the serial connection and return them.

        Return:
        -------
        str : The buffer contents
        """
        if self.s and self.s.is_open:
            try:
                if self.s.in_waiting > 0:
                    msg = ""
                    while self.s.in_waiting > 0:
                        msg += self.s.readline().decode().strip()
                        time.sleep(0.01) # Mandatory 10ms delay to prevent buffer overflow
                    return msg
            except serial.SerialException as e:
                print(f"Error reading message: {e}")

    def get_current_pose(self):
        """ Returns the current pose of the robot arm

        Message received from robot with 'get_pose' command is:
            '[10262.801][Robot]Current pose:   cords: [x: 0.13500, y: 0.00000, z: 0.21500]angles: [Roll: 0.00000, Pitch: 0.00000, Yaw:0.00000]tool: 0'

        Return:
        ----------
        np.array : A numpy array representing the current pose of the robot arm [x, y, z, roll, pitch, yaw]
        """

        # Get the pose
        self.send_message("get_pose;")

        # Read all data from the buffer
        data = self.get_buffer()

        # Extract the pose from the data
        if data:
            try:
                pose = data.split('cords: ')[1].split('angles: ')[0].strip().replace('[', '').replace(']', '').split(',')
                pose = [float(p.split(':')[1]) for p in pose]
                return np.array(pose)

            except Exception as e:
                print(f"Error extracting pose: {e}")
                return np.zeros((6,))
        else:
            print("No pose data to read from")
            return None

    def home(self):
        """ Sends the robot arm to the home position"""
        self.send_message("home;")

    def send_ctrl_c(self):
        """ Sends the Ctrl+C command to the robot arm"""
        self.s.write(b'\x03')
        time.sleep(0.01)

    def send_ctrl_d(self):
        """ Sends the Ctrl+D command to the robot arm"""
        self.s.write(b'\x04')
        time.sleep(0.01)

    def disconnect(self):
        """ A function that stops the client and closes the serial connection"""
        self.s.close()
        self.logger("Serial connection closed...")

    def set_debug_mode(self, mode):
        """ Set the debug mode of the robot arm

        Parameters:
        -----------
        mode    (bool, str): The debug mode to set (true/false or 'on'/'off'
        """
        self.send_message(f"debug:{mode}")


