
import os
#import wifi # Pico 2 doesn't have wifi yet
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

from arm_utils.armTransforms import create_circular_trajectory as circ_traj
#from arm_utils.armTransforms import create_vertical_trajectory as vert_traj
#from arm_utils.armTransforms import create_depth_trajectory as depth_traj

__author__ = "Jonathan Shulgach"
__version__ = "1.4.6"

MAXBUF = 512
QUEUE_BUFFER = 10        # Maximum number of commands to hold
MAXSTEP = 101
points = circ_traj([0.135, 0.000, 0.225], 0.05, MAXSTEP) # draw a circle
#vertical_points = create_vertical_trajectory([0.135, 0.000, 0.180], 0.04, MAXSTEP) # like a squished circle
#depth_points = create_depth_trajectory([0.1925, 0.000, 0.124], 0.05, MAXSTEP) # like a squished circle

class AsyncController(object):
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
        use_uart          : (bool)    Enable/Disable UART messasing with usb port. False requires using UART pins
        speed_control     : (bool)    Enable/Disable smooth robot joint setting
        verbose           : (bool)    Enable/disable verbose output text to the terminal
    """
    def __init__(self, name='Controller',
                       ip=None,
                       port=1000,
                       rate=100,
                       simulate_hardware=False,
                       use_wifi=False,
                       use_serial=False,
                       use_uart=False,
                       speed_control=False,
                       command_delimiter=";",
                       verbose=False,
                       ):
        self.name = name
        self.ip = ip
        self.port = port
        self.rate = rate
        self.simulate_hardware = simulate_hardware
        self.use_wifi = use_wifi
        self.use_serial = use_serial
        self.use_uart = use_uart
        self.speed_control = speed_control
        self.command_delimiter = command_delimiter
        self.verbose = verbose

        # Internal parameters
        self.counter = 0
        self.connected = False
        self.queue = []
        self.all_stop = False # Flag for running main threads
        self.run_traj = False # Flag for running trajectories
        self.stop_after_disconnect = False
        self.prev_t = 0

        # Create robot
        self.robot = RobotArm(simulate_hardware=self.simulate_hardware,
                              dh_params=robot_models.miniarm_params,
                              speed_control=self.speed_control,
                              verbose=self.verbose)

        self.robot.forward_kinematics()  # update self.config with new values.
        self.robot.home() # Set the physical robot hardware to the "home" position
        #self.robot.ikhome()

        # Create objects for the RGB LED, the music player, and the FRSs on the gripper
        self.rgb = RGBLED(set_color=[30, 0, 0])
        self.player = MusicPlayer()
        self.fsr = {'Left':ForceSensor(board.A0), 'Right':ForceSensor(board.A2)}

        # Establish peripheral connection types
        if self.use_wifi: self.server = self.connect_to_wifi()
        if self.use_serial: self.serial = USBSerialReader(use_UART=self.use_uart, TERMINATOR=self.command_delimiter, verbose=self.verbose)

        self.logo() # Show awesome logo
        #if self.verbose: self.logger("{} object created!".format(self.name))
        if self.verbose: self.display_robotinfo() # Display current robot state

    def logger(self, *argv, warning=False):
        """ Robust printing function to log events"""
        msg = ''.join(argv)
        if self.robot.warning or warning: msg = '(Warning) ' + msg
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))
        if self.robot.warning or warning: self.rgb.set_color([50,50,0]) # Yellow

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
        #asyncio.create_task( self.helper_queue_msgs( 100 ) )  # 50 

        # Start main routine
        if self.verbose: self.logger("Setting up robot update rate")
        asyncio.create_task( self.update() )

        if self.verbose: self.logger("{} running!".format(self.name))

        while self.all_stop != True:
            await asyncio.sleep(0) # Calling #async with sleep for 0 seconds allows coroutines to run

    async def update(self, interval=200):
        """ Asyncronous co-routing that updates the controller. Messages in the queue will be 'dequeued'
            depending on the command type.
        """
        while self.all_stop != True:
            # MINIARM debug commands. Uncomment below to see the FK/IK calculation checks
            #self.robot.helper_forward_kinematics([0.00000,0.00000,0.00000,0.00000,0.00000,0.00000],
            #                                     [0.1350, 0.0000, 0.2150], # pose
            #                                     [0.0000, 0.0000, 0.0000]) # euler angles
            #self.robot.helper_inverse_kinematics([0.00,0.00,0.00,0.00,0.00,0.00],
            #                                     [0.135, 0.00, 0.215], # pose
            #                                     [0.00, 0.00, 0.00]) # euler angles
            #self.robot.helper_inverse_kinematics([-5.04872683e-17,  1.77628832e-01,  5.23401762e-01, -1.72568299e-16, -7.01030594e-01,  3.53840963e-17,  3.53840963e-17],
            #                                     [0.135, 0.00, 0.155],
            #                                     [0.00, 0.00, 0.00])
            #self.robot.helper_inverse_kinematics([-2.80746871e-17,  2.83554572e-01,  2.82139363e-04,  2.85740847e-19,  1.28606746e+00, -1.35783164e-17, -1.35783164e-17],
            #                                     [0.135, 0.00, 0.155],
            #                                     [0.00,-1.57,0.00])
            #self.robot.helper_forward_kinematics([0.00000,0.00000,-np.pi/2,0.00000,0.00000,0.00000],
            #                                     [0.015, 0.0000, 0.275], # pose
            #                                     [0.0000, -np.pi/2, 0.0000]) # euler angles
            #self.robot.helper_inverse_kinematics([0.00,0.00,-np.pi/2,0.00,0.00,0.00],
            #                                    [0.015, 0.00, 0.275], # pose
            #                                    [0.00, -np.pi/2, 0.00]) # euler angles

            # KUKA debug commands. Uncomment below to see the FK/IK calculation checks (adjust dh params with kuka params) 
            #self.robot.helper_forward_kinematics([0.00,0.00,0.00,0.00,0.00,0.00],
            #                                     [2.153, 0.0000, 1.946], # pose # using the KUKA ARM
            #                                     [0.0000, 0.0000, 0.0000]) # euler angles
            #self.robot.helper_inverse_kinematics([0.00,0.00,0.00,0.00,0.00,0.00],
            #                                     [2.153, 0.00, 1.946], # pose
            #                                     [0.00, 0.00, 0.00]) # euler angles
            #self.robot.helper_forward_kinematics([1.01249809363771,  -0.275800363737724,  -0.115686651053751, 1.63446527240323,  1.52050002599430, -0.815781306199679],
            #                                     [0.49792,1.3673,2.4988],
            #                                     [0.366, -0.078, 2.561])
            #self.robot.helper_inverse_kinematics([1.01249809363771,  -0.275800363737724,  -0.115686651053751, 1.63446527240323,  1.52050002599430, -0.815781306199679],
            #                                     [0.49792,1.3673,2.4988],
            #                                     [0.366, -0.078, 2.561])
            #self.robot.helper_forward_kinematics([-0.0682697289101386, 0.434273483083027, -1.13476160607020, 0.206486955261342, 0.604353673052791, -0.0272724984420472],
            #                                     [2.3537, -0.1255546, 2.841452],
            #                                     [0.131008, -0.10541, 0.0491503])
            #self.robot.helper_inverse_kinematics([-0.0682697289101386, 0.434273483083027, -1.13476160607020, 0.206486955261342, 0.604353673052791, -0.0272724984420472],
            #                                     [2.3537, -0.1255546, 2.841452],
            #                                     [0.131008, -0.10541, 0.0491503])

            # Adding commands to the queue list is the more efficient method to command the robot but it takes more time.
            # This could be a performance bottleneck if the microcontroller isn't able to handle the necessary speeds; in 
            # which case sending serial commands to the robot directly works fastest
            if len(self.queue)>0:
                msg = self.dequeue_msg()
                if self.verbose: self.logger("Reading {} from list".format(msg))
                await self.parse_command([msg])

                # Uncomment below to see the speed in which the command parser can run at:
                #print("{}".format(1/(time.monotonic()-self.prev_t)))
                #self.update_rate = 1/(time.monotonic()-self.prev_t)
                #self.prev_t = time.monotonic()

            await asyncio.sleep(1 / int(interval))
            #await asyncio.sleep(0) # handle this as fast as possible compared to the other coroutines           
        self.logger("Main loop exited")

    async def parse_command(self, msg_list):
        self.robot.warning = False
        for msg in msg_list:
            try:
                cmd = msg[0]
                msg = [m.replace(';','') for m in msg] # Remove ';' from all parts of the message

                if cmd == 'test':         self.logger('Testing 123')           # Simple test output
                elif cmd == 'movemotor':  await self.movemotor(msg)            # single motor movement               
                elif cmd == 'movemotors': await self.movemotors(msg)           # multiple motor movement
                elif cmd == 'info':       self.display_robotinfo()             # Get the information of the robot
                elif cmd == 'set_gripper':await self.set_gripper(msg)          # Set gripper to specified angle     
                elif cmd == 'set_pose':   await self.set_pose(msg)             # Global Pose command ([x, y, z, roll, pitch, yaw])
                elif cmd == 'get_pose':   self.robot.get_pose()                # Get the current pose of the robot
                elif cmd == 'get_joints': await self.get_joints()              # Get the current joint state
                elif cmd == 'set_joints': self.logger("set_joints command disabled") # Set the robot joint state
                 # ==== Check difference ======
                elif cmd == 'set_delta_pose': await self.set_delta_pose(msg)   # Relative cartesian position step(in the form of [x, y, z, roll, pitch, yaw])
                elif cmd == 'posture':    self.set_posture_delta(msg)          # Cartesian displacement from reference point ([x, y, z, roll, pitch, yaw])                
                elif cmd == 'controller': await self.set_controller_delta(msg) # Controller command received with an array of controller input values                    
                # ===========================
                elif cmd == 'help':       await self.print_help()              # Read back instructions on supported commands
                elif cmd == 'play_music': await self.play_file(msg)            # Play music
                elif cmd == 'set_led':    self.set_led(msg)                    # Change LED color
                elif cmd == 'home':       self.robot.home()                    # Set joint positions to initial packed position                   
                elif cmd == 'ikhome':     self.robot.ikhome()                  # Set joint positions to 0
                elif cmd == 'fsr':        self.read_fsr(msg)                   # Read the sensor value from an FSR
                elif cmd == 'debug':      await self.set_verbose_mode(msg)     # Turn on/off the verbose output
                elif cmd == 'set_rate':       await self.set_loop_rate(msg)    # Update main loop rate (Hz)
                elif cmd == 'trajectory': await self.start_trajectory(msg)     # Perform specified trajectory
                elif cmd == 'stop':       await self.stop_trajectory()         # Stop ongoing trajectories
                else: self.logger("Unknown command received: '{}'".format(cmd))
            except:
                print("Unknown error")
                self.rgb.set_color([100,100,0]) # Yellow

        ## To check the actual update rate in which the controller is performing at, enable the lines below:
        #print("{}".format(1/(time.monotonic()-self.prev_t)))
        #self.update_rate = 1/(time.monotonic()-self.prev_t)
        #self.prev_t = time.monotonic()
    async def get_joints(self):
        """ Function that returns the state of all the robot joints, gripper included """
        robot_joints = self.robot.get_joints()
        robot_joints.append(self.robot.get_gripper())
        self.logger(f"{robot_joints}")

    def display_robotinfo(self):
        """ Helper function to display status of the robot """
        self.logger("\n================================ Robot Info =================================")
        self.robot.robotinfo()
        #self.read_fsr() # Enable when we have FSRs
        self.logger("Simulated hardware: {}".format(self.robot.simulate_hardware))
        self.logger("Verbose output: {}".format(self.verbose))
        self.logger("Main loop rate: {}Hz".format(self.rate))
        self.logger("\n=============================================================================")

    async def movemotor(self, cmd):
        """ Function that handles a command that updates a specified robot motor to an angle """
        if len(cmd) < 2:
            self.logger("'movemotor' command received, missing motor index and angle")
        elif len(cmd) < 3:
            self.logger("'movemotor' command and motor index received, missing motor angle (ex: 90)")
        else:
            try:    self.robot.set_joint(int(cmd[1]), int(cmd[2]))
            except: self.logger("Error in value being written to motor")

    async def start_trajectory(self, cmd):
        """ Function that performs a predefined trajectory (builtin keyword or point list)"""
        if len(cmd) < 2:
            self.logger("'trajectory' command received, missing trajectory data [list of xyz] or 'circle'")
            return
        try:
            traj = cmd[1]
            if isinstance(traj, str):
                if traj.lower() in ['circle']:
                    MAXSTEP = 101
                    traj = circ_traj([0.135, 0.000, 0.25], 0.02, MAXSTEP) # draw a circle
            repeat = False # Option to repeat trajectory, setting default to false
            if len(cmd) > 2:
                if cmd[2].lower() in ['true', 'false']: repeat = cmd[2]
                else: self.logger("Repeat option unrecognized, pass true/false")
            self.run_traj = True
            asyncio.create_task(self.run_trajectory(traj, repeat))
        except:
            self.logger("Error in setting trajectory", warning=True)

    async def stop_trajectory(self):
        """ Stops all ongoing trajectories """
        if self.verbose: self.logger("Stopping trajectory")
        self.run_traj = False

    async def run_trajectory(self, trajectory_points, repeat=False):
        """ Run a given trajectory, loops if repeat is set to true """
        index = 0
        while self.run_traj:
            for index in range(len(trajectory_points)):
                if not self.run_traj:
                    break
                pose_msg = [["set_pose", ",".join(map(str, trajectory_points[index])) + ",0.00,0.00,0.00;"]]
                await self.parse_command(pose_msg)
                await asyncio.sleep(1 / self.rate)
            if not repeat:
                break
        if self.verbose: self.logger("Trajectory completed or stopped.")

    async def movemotors(self, cmd):
        """ Function that handles a command that updates all robot motors by the order values are received """
        if len(cmd) < 2:
            self.logger("'movemotors' command received, but missing array of joint states (ex: [10, 10, 0, 0, 10, 10])")
        else:
            vals_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")
            try:
                angle_list = [float(i) for i in vals_str]
                # Manual selection of either quick set of joint positions or using the step-based speed control
                self.robot.set_joints(angle_list[:6])
                #self.robot.set_joints_speed_control(angle_list[:6], 240)
                if len(angle_list) > 6:
                    self.robot.set_gripper(int(angle_list[6]))
            except: self.logger("Error in values being written to motors")

    async def set_gripper(self, cmd):
        """ Set gripper state/angle to specified position """
        if len(cmd) < 2:
            self.logger("'gripper' command received, missing value (ex: 90, 'open')") 
        val = int(cmd[1])
        #self.robot.set_gripper(int(cmd[1]))
        self.robot.update_servo(9, val)
        #try:
        #    self.logger("Received {}".format(cmd[1]))
        #    #self.robot.set_gripper(int(float(cmd[1]))) # convert string to float before converting to int
        #    self.robot.set_joint(6, int(float(cmd[1])))
        #except:
        #    self.logger("Not a number")
        #    self.logger("Sending state")
        #    self.robot.set_gripper(cmd[1].lower())

    async def set_pose(self, cmd):
        """ Function that handles a command that updates the end effector pose to an absolute coordinate """
        if len(cmd) < 2:
            self.logger("No position data received. Expected '[x, y, z]' or '[x, y, z, roll, pitch, yaw]'")
        else:
            pose_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")
            vals = [float(i) for i in pose_str]
            print(vals)
            self.robot.set_pose(vals)

    async def set_delta_pose(self, cmd):
        """ Function that handles a command that updates the end effector pose with a delta movement """
        if len(cmd) < 2:
            self.logger("No position data received. Expected '[x, y, z]' or '[x, y, z, roll, pitch, yaw]'")
        else:
            try:
                pose_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",")
                new_pose = self.robot.handle_delta(pose_str)
                print(new_pose)
                self.robot.set_pose( new_pose )
            except:
                self.logger("Error in sending delta pose",True)

    def set_posture_delta(self, cmd):
        """ Function that handles relative displacement from an origin point """
        if len(cmd) < 2:
            self.logger("No position data received. Expected '[x, y, z]' or '[x, y, z, roll, pitch, yaw]'")
        else:
            pose_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").split(",") 
            self.robot.set_pose( self.robot.handle_posture_delta(pose_str) )

    async def set_controller_delta(self, cmd):
        """ Function that handles a controller command by converting it into a delta position, 
            followed by updating the end effector pose with that delta movement
        """
        if len(cmd) < 2:
            self.logger("Received 'controller' command but missing controller data...")
        else:
            new_delta_pos = convertControllerToDelta(cmd[1])
            if self.verbose: self.logger("New delta pose: {}".format(new_delta_pos))
            self.robot.set_pose( self.robot.handle_delta(new_delta_pos) )

    async def play_music(self, cmd):
        """ Function that handles a command to play file. Checks that the file exists before playing """
        if len(cmd) < 2:
            self.logger('Missing name of file to play')
        else:
            if cmd[1] not in self.player.songlist.keys():
                self.logger('Error in playing file. Check that name is included in library')
            else:
                self.player.play(cmd[1])

    def set_led(self, cmd):
        """ Function that sets the RGB LED color """
        if len(cmd) < 2:
            self.logger("Missing LED RGB data. Send data as array of values from 0-255 (ex: [0, 0, 100])")
        else:
            rgb_str = cmd[1].replace(" ", "").replace("[", "").replace("]", "").replace(";","").split(",")
            rgb_val = list(map(int, rgb_str))
            if self.verbose: self.logger("Setting new values: {}".format(rgb_val))
            self.rgb.set_color(rgb_val)

    async def set_verbose_mode(self, cmd):
        """ Enable/disable the verbose output by setting the attribute to true or false """
        if cmd[1].lower() in ['on', 'true']:
            self.verbose = True
        elif cmd[1].lower() in ['off', 'false']:
            self.verbose = False
        else:
            self.logger("Unknown input, acceptable inputs are on/off/true/false", warning=True)
        self.logger(f"Debug mode set to {self.verbose}")
        self.serial.verbose = self.verbose
        self.robot.verbose = self.verbose
        self.rgb.verbose = self.verbose
        self.player.verbose = self.verbose

    async def set_loop_rate(self, cmd):
        """ Update the main loop rate (Hz) """
        if len(cmd) < 2:
            self.logger("Missing rate value. Usage: rate <value>")
        try:
            self.rate = int(cmd[1])
            self.logger(f"Main loop rate updated to {self.rate} Hz")
        except ValueError:
            self.logger("Invalid rate value, please provide valid integer")

    async def print_help(self):
        """ Display supported commands"""
        self.logger('''
 ================================= List of commands =============================================
 movemotor   |  MOTOR VALUE     | // Moves motor A to absolute position B (deg)
 movemotors  |  VALUES          | // Moves motors absolute position B (deg) assimung VALUES is a list
 info        |                  | // Prints info about robot system (motors, grippers, and sensors)
 set_gripper |  VALUE, STRING   | // Gripper command to set the state (open/close) or position
 set_pose    |  VALUES          | // Updates the end effector pose to an absolute cartesian coordinate pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 get_pose    |                  | // Returns the current position and orientation of the robot end effector
 set_joints  |  (DISABLED)      | // Updates the robot joint state. Pass a list of values (ex: [0,0,0,0,0,0])
 get_joints  |                  | // Returns the current robot joint state
 set_led     |  VALUES          | // Set the RGB LED to a specific color using 0-255 values in a 3-element list
 set_delta   |  VALUES          | // Updates the end effector pose with a delta movement relative to the robot's current pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 posture     |  (DISABLED)      | // Updates the end effector pose with a Cartesian displacement relative to the robot's origin. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 controller  |  (DISABLED)      | // Controller-specific message as a long ascii string with buttons and joystick data that gets converted into a delta position. Check 'xbox_utils' for message type details
 help        |                  | // Display available commands
 play_music  |  STRING          | // Play a music file 
 debug       |  STRING          | // Pass 'on' or 'off' to enable or disable the verbose output
 set_rate    |  VALUE           | // Update the main loop rate (Hz)
 trajectory  |  STRING [REPEAT] | // Perform a specified trajectory (e.g. 'circle') with optional repeat argument (true/false)
 stop        |                  | // Stop ongoing traetories
 fsr         |                  | // Read the sensor values from the FSRs
 home        |                  | // Set the robot to its home position
 test        |                  | // Test command to verify output from device
 ================================================================================================
 ''')

    def dequeue_msg(self, idx=0):
        """Handles dequeuing of commands in the specified order """
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
        """ Read serial commands and add them to the command queue """
        while self.all_stop != True:
            self.serial.update()
            
            # If connection was lost
            if not self.serial.connected and self.connected:
                self.rgb.set_color([100, 0, 0])
                self.connected = False
            elif self.serial.connected and not self.connected:
                self.rgb.set_color([0, 100, 0])
                self.connected = True
                
            if self.serial._out_data:        
                # Skip the queueing structure and handle commands as soon as they come
                data = self.serial.out_data
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
            
            pose_msg = [["pose", ",".join([str(i) for i in points[self.counter]]) +",0.000,0.000,0.000"]]
            #pose_msg = [["pose", ",".join([str(i) for i in vertical_points[self.counter]]) +",0.00,0,0.000"]]
            #pose_msg = [["pose", ",".join([str(i) for i in depth_points[self.counter]]) +",0.00,15,0.000"]]
            #pose_msg = [["pose", "0.135,0.01,0.216,0.000,0.000,0.000"]]
            #await self.queue_msg(pose_msg)
            await self.parse_command(pose_msg)
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

    def logo(self):
        self.logger(''' 
 __  __  _  __  _  _      ____  _____  __  __ 
|  \/  || ||  \| || |    / () \ | () )|  \/  |
|_|\/|_||_||_|\__||_|   /__/\__\|_|\_\|_|\/|_|
''')
