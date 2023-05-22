import socket
import sys
from threading import Timer
from xbox_control import XboxController
import numpy as np

MESSAGE_TERMINATOR = ";"
MAXSTEP = 51
TOLERANCE = 0.1

def create_circular_trajectory(center, radius=10, steps=101):
    theta = np.linspace(0, 2*np.pi, steps)
    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
    return center + radius*temp

points = create_circular_trajectory([130, 0, 270], 40, MAXSTEP) # draw a circle
euler = np.array([0,0,0]) # keep same orientation for all points

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
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
        

class XboxClient():
    def __init__(self,
                 ip='192.168.1.165', # for the dedicated dns once it goes back up: 'desktoparm.ddns.net'
                 port=1000,
                 rate=10,
                 verbose=True,
                 ):
        """
        ControllerClient is an object that acts as a simple TCP Client and transmits the state of a connected controller for teleoperation

        Parameters:
        -----------
        node_name : (str)     A specific node name
        ip        : (str)     The IP address to use to start the server
        port      : (int)     Port number on ip address to allow client connections
        rate      : (int)     Frequency of publishing rate for data to IP address
        verbose   : (bool)    Enable/disable verbose output text to the terminal

        """
        self.ip = ip
        if len(sys.argv)>1:
            self.ip = str(sys.argv[1])
        self.port = port
        if len(sys.argv)>2:
            self.port = int(sys.argv[2])
        self.rate = rate
        self.verbose = verbose
        self.connected = False
        self.controller_exists = False
        self.counter = 0
        self.prev_data = []
        self.prev_msg = ""
        self.timer = RepeatedTimer(1/self.rate, function=self.timer_callback)

        # Create controller object with button event handling
        self.xbox = XboxController()
        self.controller_exists = True if len(self.xbox.gamepads) > 0 else print("Joystick controller created. Connect a joystick controller to begin transmitting...")
            
        # Attempt to connect to server
        if self.verbose: print('Attempting to connect to {} port {}'.format(self.ip, self.port))
        self.connect(self.ip, self.port)


    def timer_callback(self):
        """ A callback function that gets called every "1/rate" seconds
        The function reads the current state of the joystick and sends it as a string to the
        connected ip address and port

        """
        if len(self.xbox.gamepads) > 0:
            if not self.controller_exists:
                print("New controller detected!")
                self.controller_exists = True
        
            try:      
                joy_data = []
                joystick_indices = [0, 1, 2, 3, 4, 5]
                data = self.xbox.read()
                if not all( abs(i)<TOLERANCE  for i in data): 
                    for j in data:
                        if abs(j) >= TOLERANCE: joy_data.append(j)
                        else: joy_data.append(0.0)
                    #joy_data = [data[i] for i in joystick_indices]
                    
                    msg = ",".join([str(elem) for elem in joy_data])
                    msg = "controller " + msg + MESSAGE_TERMINATOR # adding the keyword for the controller command parser, and terminator
                    #msg =  "pose " + ",".join([str(i) for i in points[self.counter]]) +",0,0,0"+ MESSAGE_TERMINATOR
                    #if all(data[i]>0 for i in joystick_indices)
                    self.sock.send(msg.encode())
                    self.prev_msg = msg
                    self.prev_joy = joy_data
                    if self.verbose: print('[{}] Sent messgage: {!r}'.format(self.counter, msg))
                    #self.counter+=1
                    if self.counter >= MAXSTEP-1:
                        self.counter = 0
                    else:
                        self.counter += 1
                
            except KeyboardInterrupt:
                if self.verbose: print("KeyboardInterrupt detected")
                self.connected = False
                self.stop()        

            except socket.error:
                if self.verbose: print('Connection to the server lost, attempting to reconnect...')
                self.connected = False
                self.timer.stop()
                #self.connect(self.ip, self.port)
    

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
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a TCP/IP socket.
                self.sock.connect((ip, port))
                self.connected = True
                if self.verbose: print('Connection to server at {} successful'.format((ip, port)))
                self.timer.start()
    
            except KeyboardInterrupt:
                if self.verbose: print("KeyboardInterrupt detected")
                self.connected = True
                self.stop()
                
            except socket.error:
                self.connected = False
                self.timer.stop()

        
    def disconnect(self):
        """ Function to explicitly disconnect from microcontroller if connected and clean serial use 
        to allow new connection 
        """
        if self.sock:
            self.sock.close()
            if self.verbose: print('Socket closed')

    def stop(self):
        """ Explicit command to stop the client """
        self.timer.stop()
        self.disconnect()


if __name__ == '__main__':

    # Specify the ip address to connect to
    ip = '192.168.1.165' 
    xbox = XboxClient(ip=ip, port=1000, rate=10)
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Oh! you pressed CTRL + C. Program interrupted...")
    finally:
        xbox.stop()       
