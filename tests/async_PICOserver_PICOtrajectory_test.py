""" This script tests the speed of execution with multiple asyncronous coroutines. It sets up a server to 
    handle socket streaming and exchanging of data. Another routine updates a queue message buffer with 
    cartesian points to send to the robot as new poses
    
    Maximum update rates seen from my testing go as high as 28Hz, a little less than the values seen from 
    'PICO_trajectory_test.py'. 
    This is likely due to the time it takes to pass the numpy.array data type into the queue as the message
    gets read. This is also a minimalistic controller so the number of intermediary commands can contribute
    
    
"""

import os
import wifi
import time
import asyncio
import ulab.numpy as np
import arm_control.robotarm as robotarm

QUEUE_BUFFER = 10       # Maximum number of commands to hold
MAXBUF = 1024

wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))

def create_circular_trajectory(center, radius=10):
    theta = np.linspace(0, 2*np.pi, 201)
    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
    return center + radius*temp
        
points = create_circular_trajectory([130, 0, 270], 40) # draw a circle
euler = np.array([0,0,0]) # keep same orientation for all points


class AsyncTest:
    def __init__(self):
        self.numbers = [] # list that new numbers will get stored to
        self.robot = robotarm.RobotArm(simulate_hardware=False)
        self.all_stop = False
        self.counter = 0
        self.prev_t = 0
        self.queue = []
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
        self.robot.robotinfo()

    async def serve_client(self, reader, writer):
        """ Function that runs once a client has connected to the server"""
        print("Client connected")
        while self.all_stop != True:
            request = await reader.read(MAXBUF)
            request = ''.join([chr(b) for b in request]).replace('\x00','')
            if request == b'': break          
            print("Heard {}  from client".format(request))           
        writer.close()
        print('Server shutdown')

    def dequeue_msg(self, idx=0):
        """Handles dequeuing of commands in the specified order """
        return self.queue.pop(idx)
        
    async def queue_msg(self, msg):
        """Handles placing new requests in a queue attribute. Uses the message terminator to deal with
           a list of commands to process """
        if len(self.queue) > QUEUE_BUFFER:
            print("Max queue buffer reached")
        else:
            self.queue.append(msg)

    async def queue_msgs(self, interval):
        """ Asyncronous corouting that updates the controller. Messages in the queue will be 'dequeued'
            depending on the command type.
        """
        while self.all_stop != True:
            if self.counter >= 200:
                self.counter = 0
            else:
                self.counter += 1
            await self.queue_msg(points[self.counter])
            await asyncio.sleep(1 / int(interval))


    async def update(self):
        """ Asyncronous corouting that updates the controller. Messages in the queue will be 'dequeued'
            depending on the command type.
        """
        while self.all_stop != True:        
            if len(self.queue)>0:
                pose = np.concatenate((self.dequeue_msg(), euler), axis=0) 
                #print("pose to send: {}".format(pose))
                self.robot.set_pose(pose)
                #self.robot.robotinfo()
                print("{}".format(1/(time.monotonic()-self.prev_t)))
                self.prev_t = time.monotonic()
                
            await asyncio.sleep(0) # handle this as fast as possible compared to the other coroutines           


    async def main(self):
        """ Keeping the main tasks and coroutines in a single main function """
        print('Setting up webserver...')
        asyncio.create_task(asyncio.start_server(self.serve_client, "192.168.1.177", 1000))
        
        rate = 50
        asyncio.create_task( self.queue_msgs( rate ) ) 
        
        asyncio.create_task( self.update() ) 

        while self.all_stop != True:
            await asyncio.sleep(0) # Calling #async with sleep for 0 seconds allows coroutines to run
        
        
if __name__ == '__main__':

    obj = AsyncTest()
    try:
        asyncio.run(obj.main()) # Need to pass the async function into the run method to start
    except:
        obj.all_stop = True