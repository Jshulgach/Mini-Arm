
import os
import wifi
import time
import board
import asyncio
import usb_cdc
import ulab.numpy as np
import supervisor
from rgbled import RGBLED
from musicplayer import MusicPlayer
from arm_utils import robot_models
from arm_utils.xbox_utils import *
from arm_control.robotarm import RobotArm
from arm_utils.force_sensor import ForceSensor
from arm_utils.usbserialreader import USBSerialReader

__author__ = "Jonathan Shulgach"
__version__ = "1.0.1"

MAXBUF = 512
MAXSTEP = 101
COMMAND_DELIMITER = ";"  # multiple commands should be parsed too
ARGUMENT_DELIMITER = ":" # expecting arguements to be spaced
QUEUE_BUFFER = 10       # Maximum number of commands to hold

CIRCULAR_TEST = False

from arm_utils.armTransforms import create_circular_trajectory
points = create_circular_trajectory([1.75300, 0.000, 0.746], 0.50, MAXSTEP) # draw a circle
euler = np.array([0.000,0.000,0.000]) # keep same orientation for all points

    

class AsyncController:
    """This is the object that handles serial command inputs and directs the servo positions
        according to the commands given.

        Parameters:
        -----------
        name              : (str)     The custom name for the class
        ip                : (str)     The IP address to use to start the server
        port              : (int)     Port number on ip address to allow client connections
        rate              : (int)     Frequency of publishing rate for data to IP address
        simulate_hardware : (bool)    Allow physical control of hardware
        use_wifi          : (bool)    Enable/Disable server
        use_serial        : (bool)    Enable/Disable serial command parsing
        verbose           : (bool)    Enable/disable verbose output text to the terminal
    """    
    def __init__(self, name='Controller', 
                       ip=None, 
                       port=1000, 
                       rate=100, 
                       simulate_hardware=False, 
                       use_wifi=False, 
                       use_serial=False, 
                       verbose=False,
                       ):
        self.name = name
        self.ip = ip
        self.port = port
        self.rate = rate
        self.use_wifi = use_wifi
        self.use_serial = use_serial
        self.verbose = verbose
        
        # Internal parameters
        self.counter = 0
        self.connected = False
        self.queue = []
        self.all_stop = False
        self.stop_after_disconnect = False
        self.prev_t = 0
        
        # Create robot
        self.robot = RobotArm(simulate_hardware=simulate_hardware, verbose=self.verbose)
        self.robot.dh = robot_models.kuka_params
        self.robot.dh = robot_models.miniarm_params
        self.robot.forward_kinematics()  # update self.config with new values.
        
        # Create objects for the RGB LED, the music player, and the FRSs on the gripper
        self.rgb = RGBLED(set_color=[30, 0, 0])
        self.player = MusicPlayer()
        self.fsr = {'Left':ForceSensor(board.A0), 'Right':ForceSensor(board.A2)}
        
        # Establish peripheral connection types
        if self.use_wifi: 
            self.server = self.connect_to_wifi()        
        if self.use_serial:
            self.usb_serial = USBSerialReader(';', verbose=self.verbose)
            
        if self.verbose: self.logger("{} object created!".format(self.name))
        
        # Display current robot state
        if self.verbose: self.display_robotinfo()
  
    async def main(self):
        """ Start main tasks and coroutines in a single main function """
        
        if self.use_serial:
            if self.verbose: self.logger("USB Serial Parser set up. Reading serial commands")
            asyncio.create_task(self.serial_client(30))
        
        # Routine to receive network commands. Largs strings bog down queueing message handler.
        if self.use_wifi:
            self.logger("Setting up webserver on ( ip: '{}', Port: {} ) ...".format(str(wifi.radio.ipv4_address), self.port))
            asyncio.create_task(asyncio.start_server(self.serve_client, str(wifi.radio.ipv4_address), self.port))
        
        # Helper routine to keep publishing messages to the queue instead of waiting on a client, disable after debug
        if CIRCULAR_TEST: asyncio.create_task( self.helper_queue_msgs( 50 ) )  # 50
        
        if self.verbose: self.logger("Setting up robot update rate")
        asyncio.create_task( self.update() )

        if self.verbose: self.logger("{} running!".format(self.name))
         
        while self.all_stop != True:
            await asyncio.sleep(0) # Calling #async with sleep for 0 seconds allows coroutines to run

    async def update(self, interval=100):
        """ Asyncronous co-routing that updates the controller. Messages in the queue will be 'dequeued'
            depending on the command type.
        """
        while self.all_stop != True:
            # Using the queue list is more efficient but takes more time. Uncomment below to 
            # wait for messages to be dequeued before parsing them
            if len(self.queue)>0:
                msg = self.dequeue_msg()
                if self.verbose: self.logger("Reading {} from list".format(msg))
                await self.parse_command(msg)
            
            # Waiting for serial commands directly instead of queueing them will be faster
            # ========== temporary testing ===========
                #pose = np.concatenate((msg, euler), axis=0) 
                #print("pose to send: {}".format(pose))
                #self.robot.set_pose(pose)
                #self.robot.robotinfo()
                
                # To check the actual update rate in which the controller is performing at, enable the lines below:
                #if self.verbose: print("{}".format(1/(time.monotonic()-self.prev_t)))
                #self.prev_t = time.monotonic()
                
                
            #self.robot.set_joints([0, 0, 0, 0, 90, 0])
            #self.robot.direct_kinematics()
            #self.robot.robotinfo()
            #self.robot.helper_direct_kinematics([0, 0, 0, 0, 0, 0], [185, 0, 215], [0, 90, 0])
            #self.robot.helper_direct_kinematics([1.01249809, -0.27580036, -0.11568865, 1.63446527, 1.52050003, -0.81578131], 
            #                                    [0.49792, 1.3673, 2.4988], # pose
            #                                    [0.366, -0.078, 2.561]) # euler angles
            #self.robot.helper_inverse_kinematics([1.01249809, -0.27580036, -0.11568865, 1.63446527, 1.52050003, -0.81578131], 
            #                                    [0.49792, 1.3673, 2.4988], # pose
            #                                    [0.366, -0.078, 2.561]) # euler angles
            #self.robot.helper_inverse_kinematics([0.00,0.00,0.00,0.00,0.00,0.00], 
            #                                    [2.153, 0.00, 1.946], # pose
            #                                    [0.00, 0.00, 0.00]) # euler angles
            #self.robot.helper_forward_kinematics([0.00000,0.00000,0.00000,0.00000,0.00000,0.00000], 
            #                                     [2.1530, 0.0000, 1.9460], # pose
            #                                     [0.0000, 0.0000, 0.0000]) # euler angles
            #self.robot.helper_inverse_kinematics([0, 0, 0, 0, 0, 0], [185, 0, 215], [0, 90, 0])
            #self.robot.helper_inverse_kinematics([1.015, -0.2758, -0.1157, 1.6345, 1.5205, -0.8158], [0.49792, 1.3673, 2.4988], [0.366, -0.078, 2.561])
            #await self.queue_msg('pose ' + str(1755 + self.counter) + ',0,2660,0,0,0')
            #await self.queue_msg('delta 0,0,-1,0,0,0')
            #self.counter -= 1  
            # =================================================
               
            await asyncio.sleep(1 / int(interval))
            #await asyncio.sleep(0) # handle this as fast as possible compared to the other coroutines           
        self.logger("Updating stopped")

    async def parse_command(self, msg):
        if self.verbose: self.logger("Parsing command")
        #try:
        if True:
            cmd = msg[0]
            if cmd == 'test':
                # =====================================================================================
                # Simple test output
                # =====================================================================================
                self.logger('Testing 123')
            
            elif cmd == 'movemotor':
                # =====================================================================================
                # single motor movement
                # =====================================================================================
                await self.movemotor(msg)
            
            elif cmd == 'movemotors':
                # =====================================================================================
                # multiple motor movement
                # =====================================================================================
                await self.movemotors(msg)

            elif cmd == 'robotinfo':
                # =====================================================================================
                # Get the information of the robot
                # =====================================================================================
                self.display_robotinfo()

            elif cmd == 'gripper':
                # =====================================================================================
                # Set gripper to specified angle
                # =====================================================================================
               await self.movegripper(msg)
 
            elif cmd == 'pose':
                # =====================================================================================
                # absolute cartesian position (in the form of [x, y, z, roll, pitch, yaw])
                # =====================================================================================
                await self.set_pose(msg)

            elif cmd == 'delta':
                # =====================================================================================
                # Relative cartesian position step(in the form of [x, y, z, roll, pitch, yaw])
                # =====================================================================================
                await self.set_delta(msg)
            
            elif cmd == 'posture':
                # =====================================================================================
                # Relative cartesian displacement from reference point (in the form of [x, y, z, roll, pitch, yaw])
                # =====================================================================================
                self.set_posture_delta(msg)
            
            elif cmd == 'controller':
                # =====================================================================================
                # Controller command received with an array of controller input values
                # =====================================================================================
                await self.set_controller_delta(msg)
            
            elif cmd == 'help':
                # =====================================================================================
                # Read back instructions on supported commands
                # =====================================================================================
                await self.print_help()

            elif cmd == 'play':
                # =====================================================================================
                # Play music
                # =====================================================================================
                await self.play_file(msg)

            elif cmd == 'led':
                # =====================================================================================
                # Change LED color
                # =====================================================================================
                self.set_led(msg)

            elif cmd == 'home':
                # =====================================================================================
                # Set joint positions to initial position
                # =====================================================================================
                self.robot.set_joints([0.00,0.00,0.00,0.00,0.00,0.00])
                
            elif cmd == 'fsr':
                # =====================================================================================
                # Read the sensor value from an FSR
                # =====================================================================================
                self.read_fsr(msg)            
            
            elif cmd == 'debug':
                # =====================================================================================
                # Turn on or off the verbose output
                # =====================================================================================
                await self.set_verbose_mode(msg)
            
            else:
                self.logger("Unknown command received: '{}'".format(cmd))
                
        #except:
        #    print("Unknown error")

    def display_robotinfo(self):
        """ 
        """
        self.logger("\n=================================== Robot Info ===================================")
        self.robot.robotinfo()
        self.read_fsr()
        self.logger("\n==================================================================================")
        
    async def movemotor(self, cmd):
        """ Function that handles a command that updates a specified robot motor to an angle
        
        :param cmd: (list) Command message as a list containing the main command and arguments 
        """
        if len(cmd) < 2:
            self.logger("'movemotor' command received, missing motor index and angle")
        elif len(cmd) < 3:
            self.logger("'movemotor' command and motor index received, missing motor angle (ex: 90)")
        else:    
            self.robot.set_joint(int(cmd[1]), int(cmd[2]))
            
    async def movemotors(self, cmd):
        """ Function that handles a command that updates all robot motors by the order values are received
        
        :param cmd: (list) Command message as a list containing the main command and arguments 
        """
        if len(cmd) < 2:
            self.logger("'movemotors' command received, but missing array of joint states (ex: [10, 10, 0, 0, 10, 10])")
        else:
            try:
                vals_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")
                self.robot.set_joints([float(i) for i in vals_str])
            except:
                self.logger("'movemotors' aray input received, (ex: [10, 10, 0, 0, 10, 10])")

    async def movegripper(self, cmd):
        """ Function that handles a command that updates all robot motors by the order values are received
        
        :param cmd: (list) Command message as a list containing the main command and arguments 
        """
        if len(cmd) < 2:
            self.logger("'gripper' command received, missing angle (ex: 90)")
        else:
            try: 
                val = int(float(cmd[1])) # convert string to float before converting to int
                self.robot.set_gripper(val)
            except: 
                self.logger("Warning, value passed {} is not a number".format(cmd[1]))
    
    async def set_pose(self, cmd):
        """ Function that handles a command that updates the end effector pose to an absolute coordinate
        
        :param cmd: (list) Command message as a list containing the main command and arguments 
        """
        if len(cmd) < 2:
            self.logger("No position data received. Expected '[x, y, z]' or '[x, y, z, roll, pitch, yaw]'")
        else:
            pose_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")            
            self.robot.set_pose(pose_str)
            if self.verbose: self.logger("Servos: {}".format([angle.deg for angle in self.robot.angles]))

    async def set_delta(self, cmd):
        """ Function that handles a command that updates the end effector pose with a delta movement
        
        :param cmd: (list) Command message as a list containing the main command and arguments 
        """
        if len(cmd) < 2:
            self.logger("No position data received. Expected '[x, y, z]' or '[x, y, z, roll, pitch, yaw]'")
        else:
            pose_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")    
            self.robot.set_pose( self.robot.handle_delta(pose_str) )
            if self.verbose: self.logger("Servos: {}".format([angle.deg + self.robot.joint_offsets[i] for i, angle in enumerate(self.robot.angles)]))

    def set_posture_delta(self, cmd):
        """ Function that handles relative displacement from an origin point
        """
        if len(cmd) < 2:  
            self.logger("No position data received. Expected '[x, y, z]' or '[x, y, z, roll, pitch, yaw]'")
        else:
            pose_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",") 
            self.robot.set_pose( self.robot.handle_posture_delta(pose_str) )
            if self.verbose: self.logger("Servos: {}".format([angle.deg + self.robot.joint_offsets[i] for i, angle in enumerate(self.robot.angles)]))

    async def set_controller_delta(self, cmd):
        """ Function that handles a controller command by converting it into a delta position, 
            followed by updating the end effector pose with that delta movement
        
            :param cmd: (list) Command message as a list containing the main command and arguments 
        """
        if len(cmd) < 2:
            self.logger("Received 'controller' command but missing controller data...")
        else:    
            new_delta_pos = convertControllerToDelta(cmd[1])
            if self.verbose: self.logger("New delta pose: {}".format(new_delta_pos))
            self.robot.set_pose( self.robot.handle_delta(new_delta_pos) )
            if self.verbose: self.logger("Servos: {}".format([angle.deg + self.robot.joint_offsets[i] for i, angle in enumerate(self.robot.angles)]))

    async def play_file(self, cmd):
        """ Function that handles a command to play file. Checks that the file exists before playing
        
        :param cmd: (list) Command message as a list containing the main command and arguments 
        """
        if len(cmd) < 2:
            self.logger('Missing name of file to play')
        else:
            if cmd[1] not in self.player.songlist.keys():
                self.logger('Error in playing file. Check that name is included in library')
            else:
                self.player.play(cmd[1])

    def set_led(self, cmd):
        """ Function that sets the RGB LED color        
        """
        if len(cmd) < 2:
            self.logger("Missing LED RGB data. Send data as array of values from 0-255 (ex: [0, 0, 100])")
        else:
            rgb_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")
            rgb_val = list(map(int, rgb_str))
            self.rgb.set_color(rgb_val)

    async def set_verbose_mode(self, cmd):
        """ Enable/disable the verbose output by setting the attribute to true or false
        """
        if cmd[1].lower() is not 'on' and cmd[1].lower() is not 'off':
            self.logger("Warning, unknown input '{}'. Please pass 'on' or 'off'".format(cmd[1]))
        elif  cmd[1].lower() == 'on': 
            self.verbose = True
            self.logger("Debug mode set to on")
        elif  cmd[1].lower() == 'off': 
            self.verbose = False
            self.logger("Debug mode set to off")

        self.usb_serial.verbose = self.verbose
        self.robot.verbose = self.verbose
        self.rgb.verbose = self.verbose
        self.player.verbose = self.verbose
        
    async def print_help(self):
        """ Display supported commands"""
        self.logger('''\n ================================= List of commands =============================================
 movemotor MOTOR VALUE   // Moves motor A to absolute position B (deg)
 movemotors VALUES       // Moves motors absolute position B (deg) assimung VALUES is a list
 robotinfo               // Prints information about the robot system, including motors, grippers, and sensors
 gripper VALUE           // Gripper command to send the state opened (1) or closed (0)
 pose VALUES             // Updates the end effector pose to an absolute cartesian coordinate pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 delta VALUES            // Updates the end effector pose with a delta movement relative to the robot's current pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 controller              // Controller-specific message as a long ascii string with buttons and joystick data that gets converted into a delta position. Check 'xbox_utils' for message type details
 help                    // Display available commands
 play STRING             // Play a music file 
 led VALUES              // Set the RGB LED to a specific color using 0-255 values in a 3-element list
 debug STRING            // Pass 'on' or 'off' to enable or disable the verbose output
 posture VALUES          // Updates the end effector pose with a delta movement relative to the robot's origin. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
                    '''
                   )

    def dequeue_msg(self, idx=0):
        """Handles dequeuing of commands in the specified order 
        """
        return self.queue.pop(idx)
        
    async def queue_msg(self, msg):
        """Handles placing new requests in a queue attribute. Uses the message terminator to deal with
           a list of commands to process 
        """
        commands = msg.split(COMMAND_DELIMITER) 
        for command in commands:
            if len(self.queue) > QUEUE_BUFFER:
               self.logger("Max queue buffer reached")
            else:
                if command != '':
                    cmd = command.split(ARGUMENT_DELIMITER, 1) # Each new command precedes a space, followed by comma-separated arguments      
                    if self.verbose: self.logger("Adding {} to queue:".format(cmd))
                    self.queue.append(cmd) 
        
    def connect_to_wifi(self):
        """ Makes an attempt to connect to the network given the credentials.
        
        Note: Since we are using the asyncronous server initialization, we don't need to refer to an object. We can just pass a boolean when 
        the connection is successful or not
        """
        try:
            if self.verbose: self.logger("Attempting to connect to the network...")
            wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))
            self.ip = str(wifi.radio.ipv4_address)
            self.rgb.set_color([0, 100, 0])
            if self.verbose: self.logger("Connection to local network successful")
            return True
        except:
            self.logger("Warning: Error with connecting to wifi")
            return False        
    
    async def serial_client(self, interval=100):
        """ Read serial commands and add them to the command queue 
        """
        while self.all_stop != True:
            self.usb_serial.update()
            
            # If connection was lost
            if not self.usb_serial.connected and self.connected:
                self.rgb.set_color([100, 0, 0])
                self.connected = False
            elif self.usb_serial.connected and not self.connected:
                self.rgb.set_color([0, 100, 0])
                self.connected = True
                
            if self.usb_serial._out_data:        
                # Skip the queueing structure and handle commands as soon as they come
                data = self.usb_serial.out_data
                await self.parse_command(data)

            
            #await asyncio.sleep(1 / int(interval))
            await asyncio.sleep(0)

    async def serve_client(self, reader, writer):
        """ A callback function that gets called whenever a new client connection is established. It receives a 
            (reader, writer) pair as two arguments, instances of the StreamReader and StreamWriter classes. 
            Any messages received will then be 'queued' for handling by the controller. 
        """
        self.logger("Client {} connected".format(reader.e['peername']))
        self.rgb.set_color([0, 0, 100])
        request = ""
        while True:
            request = await reader.read(MAXBUF)
            #request = request.decode()
            request = ''.join([chr(b) for b in request]).replace('\x00','')
            #print("message from {}: {}".format(reader.e['peername'], request))
            if request == '': break
            await self.queue_msg(request)
                
        self.logger("Client {} disconnected".format(reader.e['peername']))
        self.rgb.set_color([0, 100, 0])
        writer.close()
        if self.stop_after_disconnect:
            self.logger('Server shutdown')
            self.rgb.set_color([30, 0, 0])
            self.stop()
            
    def logger(self, *argv):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def start(self):
        """ Makes a call to the asyncronous library to run a main routine """
        asyncio.run(self.main())  # Need to pass the async function into the run method to start

    def stop(self):
        """ Sets a flag to stop running all tasks """
        self.all_stop = True

    async def helper_queue_msgs(self, interval):
        """ Asyncronous coroutine that updates the controller. Messages in the queue will be 'dequeued'
            depending on the command type.
            
            construction of these messages may slow down the controller (from queueing a list to creating a whole
            message it drops the update function rate from 27Hz to 25Hz
        """
        while self.all_stop != True:
            if self.counter >= MAXSTEP-1:
                self.counter = 0
            else:
                self.counter += 1
            
            pose_msg = "pose:" + ",".join([str(i) for i in points[self.counter]]) +",0.000,0.000,0.000"
            await self.queue_msg(pose_msg)
            await asyncio.sleep(1 / int(interval))
            
    def read_fsr(self, cmd=None):
        """
        """
        if not cmd or len(cmd) < 2:
            self.logger("FSR Left: {}, Right: {}".format(self.fsr['Left'].newtons, self.fsr['Right'].newtons))
        else:
            key_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")
            if (key_str != 'Left') or (key_str != 'Right'):
                self.logger("Warning - invalid key. Choose 'Left' or 'Right'".format(key_str))
                return
            self.logger("Left: {}, Right: {}".format(self.fsr[cmd[1]].newtons))
