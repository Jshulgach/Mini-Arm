"""
 (c) Jonathan Shulgach - Cite and Notice license:
   All modifications to this code or use of it must include this notice and give credit for use.
   Credit requirements:
    All publications using this code must cite all contributors to this code.
    A list must be updated below indicating the contributors alongside the original or modified code appropriately.
    All code built on this code must retain this notice. All projects incorporating this code must retain this license text alongside the original or modified code.
    All projects incorporating this code must retain the existing citation and license text in each code file and modify it to include all contributors.
    Web, video, or other presentation materials must give credit for the contributors to this code, if it contributes to the subject presented.
    All modifications to this code or other associated documentation must retain this notice or a modified version which may only involve updating the contributor list.
    Primary Authors:
      - Jonathan Shulgach, PhD Student - Neuromechatronics Lab, Carnegie Mellon University
      
"""

import os
import wifi
import time
import asyncio
from rgbled import RGBLED
from musicplayer import MusicPlayer
from arm_utils.xbox_utils import *
from arm_control.robotarm import RobotArm

__author__ = "Jonathan Shulgach"
__version__ = "0.5.9"

MAXBUF = 1024
COMMAND_DELIMITER = ";"  # multiple commands should be parsed too
ARGUMENT_DELIMITER = " " # expecting arguemnst to be spaced
QUEUE_BUFFER = 100       # Maximum number of commands to hold

class AsyncController:
    def __init__(self, name='Control', ip=None, port=1000, rate=40, simulate_hardware=False, offline=False, verbose=False):
        """This is the object that handles serial command inputs and directs the servo positions
        according to the commands given.

        Parameters:
        -----------
        name        : (str)     The custom name for the class
        ip          : (str)     The IP address to use to start the server
        port        : (int)     Port number on ip address to allow client connections
        rate        : (int)     Frequency of publishing rate for data to IP address
        simulate    : (bool)    Allow physical control of hardware
        verbose     : (bool)    Enable/disable verbose output text to the terminal
        """
        self.name = name
        self.ip = ip
        self.port = port
        self.rate = rate
        self.offline = offline
        self.verbose = verbose
        
        self.counter = 0
        self.queue = []
        self.all_stop = False
        self.stop_after_disconnect = False
        self.prev_t = 0
        
        # Create robot
        self.robot = RobotArm(simulate_hardware=simulate_hardware, verbose=self.verbose) 

        # Set the transform matrix linkage dimensions in terms of x,y,z in mm        
        self.robot.a1z = 50 
        self.robot.a2x = 0 
        self.robot.a2z = 40 
        self.robot.ax2 = 10 
        self.robot.a3z = 120 
        self.robot.a4z = 0 
        self.robot.a4x = 90 
        self.robot.a5x = 30 
        self.robot.a6x = 50 
        self.robot.direct_kinematics()  # update self.config with new values.
        
        self.rgb = RGBLED(set_color=[30, 0, 0])
        self.player = MusicPlayer()
        self.connected = self.connect_to_wifi()
        if self.verbose: self.logger("{} object created!".format(self.name))
        
        # Display current robot state
        self.robot.robotinfo()
        

    def logger(self, *argv):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def start(self):
        """Makes a call to the asyncronous library to run a main routing. Dpeendent on network connection
        """
        asyncio.run(self.main()) # Need to pass the async function into the run method to start

    async def main(self):
        """ Keeping the main tasks and coroutines in a single main function """
        
        # Set up all coroutines here, like launching new threads
        if not self.offline:
            self.logger("Setting up webserver on ( ip: '{}', Port: {} ) ...".format(str(wifi.radio.ipv4_address), self.port))
            asyncio.create_task(asyncio.start_server(self.serve_client, str(wifi.radio.ipv4_address), self.port))

        self.logger("Setting up robot update rate")
        asyncio.create_task( self.update( self.rate ) ) 
        
        # Any others?
        # maybe a new task that actually updates the servos to fix the jumpiness?

        self.logger("{} running!".format(self.name))
        while self.all_stop != True:
            await asyncio.sleep(0) # Calling #async with sleep for 0 seconds allows coroutines to run


    def stop(self):
        """Sets a flag to stop running all tasks
        """
        self.all_stop = True
    
    def connect_to_wifi(self):
        """ Makes an attempt to connect to the network given the credentials"""
        try:
            self.logger("Attempting to connect to the network...")
            wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))
            self.ip = str(wifi.radio.ipv4_address)
            self.rgb.set_color([0, 100, 0])
            if self.verbose: self.logger("Connection to local network successful")
            return True
        except:
            self.logger("Warning: Error with connecting to wifi")
            return False        

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
            if len(self.queue) <= QUEUE_BUFFER:
                if command != '':
                    #print("command received: '{}'".format(command))
                    cmd = command.split(ARGUMENT_DELIMITER) # Each command will have arguments separated by             
                    if self.verbose: self.logger("Adding {} to queue:".format(cmd))
                    self.queue.append(cmd)
            else:
                self.logger("Max queue buffer reached")

    async def serve_client(self, reader, writer):
        """ An asyncronous corouting that handles client connections to the server. Any messages received 
            will be 'queued' for handling by the controller.
        """
        self.logger("Client connected")
        self.rgb.set_color([0, 0, 100])
        while True:
            request = await reader.read(MAXBUF)
            request = ''.join([chr(b) for b in request]).replace('\x00','')
            if request == '': break
            await self.queue_msg(request)
        self.logger("Client disconnected")
        self.rgb.set_color([0, 100, 0])
        writer.close()
        if self.stop_after_disconnect:
            self.logger('Server shutdown')
            self.rgb.set_color([30, 0, 0])
            self.stop()

    async def update(self, interval):
        """ Asyncronous corouting that updates the controller. Messages in the queue will be 'dequeued'
            depending on the command type.
            
        """
        while self.all_stop != True:
        
            if len(self.queue)>0:
                msg = self.dequeue_msg()
                if self.verbose: self.logger("Reading {} from list".format(msg))
                await self.parse_command(msg)
                
            if self.verbose: self.logger("Queued messages: {}".format(self.queue))
            await asyncio.sleep(1 / int(interval))  # Don't forget the await!     

            # ========== temporary testing ===========
            #self.robot.helper_direct_kinematics([0, 0, 0, 0, 0, 0], [1755, 0, 2660], [0, 0, 0])
            #self.robot.helper_inverse_kinematics([], [1755, 0, 2660 + self.counter], [0, 0, 0])
            #await self.queue_msg('pose ' + str(1755 + self.counter) + ',0,2660,0,0,0')
            
            #await self.queue_msg('delta 0,0,-1,0,0,0')
            #self.counter -= 1            
            #self.logger("{:.2}".format(1/(time.monotonic()-self.prev_t)))
            #self.prev_t = time.monotonic()
            #await asyncio.sleep(0)            
            
            # ========================================

        self.logger("Updating stopped")

    async def parse_command(self, msg):
        cmd = msg[0]
        if cmd == 'movemotor':
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
            self.robot.robotinfo()

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
            # Relative cartesian position (in the form of [x, y, z, roll, pitch, yaw])
            # =====================================================================================
            await self.set_delta(msg)
            
        elif cmd == 'controller':
            # =====================================================================================
            # Controller command received with an array of controller input values
            # =====================================================================================
            await self.set_controller_delta(msg)
            
        
        elif cmd == 'play':
            # =====================================================================================
            # Play music
            # =====================================================================================
            await self.play_file(msg)

        else:
            self.logger("Unknown command received: '{}'".format(cmd))

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
            self.robot.set_gripper(int(cmd[1]))
    
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
            self.robot.set_pose(self.robot.handle_delta(pose_str))
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
            self.logger("New delta pose: {}".format(new_delta_pos))
            self.robot.set_pose(self.robot.handle_delta(new_delta_pos))
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
    
