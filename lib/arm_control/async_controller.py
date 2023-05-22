
import os
import wifi
import time
import asyncio
import ulab.numpy as np
from rgbled import RGBLED
from musicplayer import MusicPlayer
from arm_utils.xbox_utils import *
from arm_control.robotarm import RobotArm

__author__ = "Jonathan Shulgach"
__version__ = "0.9.1"

MAXBUF = 512
MAXSTEP = 101
COMMAND_DELIMITER = ";"  # multiple commands should be parsed too
ARGUMENT_DELIMITER = " " # expecting arguements to be spaced
QUEUE_BUFFER = 10       # Maximum number of commands to hold

from arm_utils.armTransforms import create_circular_trajectory
points = create_circular_trajectory([130, 0, 270], 40, MAXSTEP) # draw a circle
euler = np.array([0,0,0]) # keep same orientation for all points


class AsyncController:
    def __init__(self, name='Controller', ip=None, port=1000, rate=100, simulate_hardware=False, offline=False, verbose=False):
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
        
        # Internal parameters
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
        """ Makes a call to the asyncronous library to run a main routine """
        asyncio.run(self.main()) # Need to pass the async function into the run method to start

    def stop(self):
        """ Sets a flag to stop running all tasks """
        self.all_stop = True

    async def main(self):
        """ Keeping the main tasks and coroutines in a single main function """
        if not self.offline:
            self.logger("Setting up webserver on ( ip: '{}', Port: {} ) ...".format(str(wifi.radio.ipv4_address), self.port))
            asyncio.create_task(asyncio.start_server(self.serve_client, str(wifi.radio.ipv4_address), self.port))
        
        # Helper routine to keep publishing messages to the queue instead of waiting on a client
        #asyncio.create_task( self.helper_queue_msgs( 50 ) ) 
        
        self.logger("Setting up robot update rate")
        asyncio.create_task( self.update(12) ) 

        self.logger("{} running!".format(self.name))
        
        while self.all_stop != True:
            await asyncio.sleep(0) # Calling #async with sleep for 0 seconds allows coroutines to run

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
            
            pose_msg = "pose " + ",".join([str(i) for i in points[self.counter]]) +",0,0,0"
            await self.queue_msg(pose_msg)
            await asyncio.sleep(1 / int(interval))

    async def update(self, interval=100):
        """ Asyncronous corouting that updates the controller. Messages in the queue will be 'dequeued'
            depending on the command type.
        """
        while self.all_stop != True:      
            if len(self.queue)>0:
                msg = self.dequeue_msg()
                if self.verbose: self.logger("Reading {} from list".format(msg))
                await self.parse_command(msg)

            # ========== temporary testing ===========
                #pose = np.concatenate((msg, euler), axis=0) 
                #print("pose to send: {}".format(pose))
                #self.robot.set_pose(pose)
                #self.robot.robotinfo()
                
                print("{}".format(1/(time.monotonic()-self.prev_t)))
                self.prev_t = time.monotonic()
                self.robot.robotinfo()
            
            #self.robot.set_joints([0, 0, 0, 0, 90, 0])
            #self.robot.direct_kinematics()
            #self.robot.robotinfo()
            #self.robot.helper_direct_kinematics([0, 0, 0, 0, 0, 0], [120, 0, 160], [0, 90, 0])
            #self.robot.helper_inverse_kinematics([], [120, 0, 160], [0, 90, 0])
            #await self.queue_msg('pose ' + str(1755 + self.counter) + ',0,2660,0,0,0')
            #await self.queue_msg('delta 0,0,-1,0,0,0')
            #self.counter -= 1  
            # =================================================
               
            await asyncio.sleep(1 / int(interval))
            #await asyncio.sleep(0) # handle this as fast as possible compared to the other coroutines           
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
            if self.verbose: self.logger("New delta pose: {}".format(new_delta_pos))
            target_pos = self.robot.handle_delta(new_delta_pos)
            self.robot.set_pose(target_pos )
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
     
    def dequeue_msg(self, idx=0):
        """Handles dequeuing of commands in the specified order """
        return self.queue.pop(idx)
        
    async def queue_msg(self, msg):
        """Handles placing new requests in a queue attribute. Uses the message terminator to deal with
           a list of commands to process """
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
        """ Makes an attempt to connect to the network given the credentials"""
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
    