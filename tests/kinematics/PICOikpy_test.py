""" This script tests out the trajectory planning of the robot arm and sending it a list of 
points to meet in cartesian space. Assumes it's using new async version of the controller
"""
import time
import ulab.numpy as np
from miniarm.armTransforms import *
from miniarm.robot import Robot


robot = Robot(simulate=True, verbose=True)
print(robot.arm_chain)

# ===== IK test ( draw straight line) =====
cart_path =  np.zeros((50,3)) 
cart_path[:,0] = 0.1 # keep fixed distance away from origin in x axis
cart_path[:,1] = np.linspace(0.0,0.20,50) # z axis movement

for target_xyz in cart_path:
    # Calculate the new joint state from the pose goal
    new_joints = robot.set_pose(target_xyz, [0,90,0])
    print("new joints")
    print(new_joints)
    print("New joints (deg):")
    print( ["{:.2f} ".format(rad2degree(i)) for i in new_joints])


