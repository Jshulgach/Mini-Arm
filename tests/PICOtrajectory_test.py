# This is a test of the servo capabilities from the Adafruit ServoKit and being able to update the servos at
# a quick enough rate so using the IK solver, draw a circle in cartesian space as smooth as possible

# So far the fastest average rate I've seen running pose commands to the robot: 26Hz 
#
#

import time
import ulab.numpy as np
import arm_control.robotarm as robotarm

robot = robotarm.RobotArm(simulate_hardware=False)
robot.a1z = 50 
robot.a2x = 0 
robot.a2z = 40 
robot.ax2 = 10 
robot.a3z = 120 
robot.a4z = 0 
robot.a4x = 90 
robot.a5x = 30 
robot.a6x = 50 
robot.direct_kinematics()  # update self.config with new values.
robot.robotinfo()

def create_circular_trajectory(center, radius=10):
    theta = np.linspace(0, 2*np.pi, 201)
    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
    return center + radius*temp
        
points = create_circular_trajectory([150, 0, 260], 40) # draw a circle
euler = np.array([0,0,0]) # keep same orientation for all points

prev_t = 0
while True:
    for point in points:
        pose = np.concatenate((point, euler), axis=0) 
        #print("pose to send: {}".format(pose))
        robot.set_pose(pose)
        #robot.robotinfo()
        print("{}".format(1/(time.monotonic()-prev_t)))
        prev_t = time.monotonic()
        #time.sleep(0.025)
