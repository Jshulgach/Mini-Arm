# Quick test to make sure the ikpy module works on the Windows PC. This is compared to the results
# with the old custom solver
import os
import sys
os.chdir('../lib')
sys.path.append(os.getcwd())
from miniarm.armTransforms import *

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from numpy import linspace, zeros
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def update_pose(xyz, rpy):
    r""" calculate new joint state positions """
    rpy = eulerAnglesToRotMatrix(degree2rad(rpy[0]), degree2rad(rpy[1]), degree2rad(rpy[2]))
    new_joints = arm_chain.inverse_kinematics(
        target_position=xyz,
        target_orientation=rpy,
        orientation_mode='all',
    )        
    return new_joints
    
# Build robot chain
arm_chain = Chain(name='arm', links=[
    OriginLink(),
    URDFLink(
      name="Base",
      origin_translation=[0, 0, 0.050],
      origin_orientation=[0, 0,0],
      rotation=[0, 0, 1],
      bounds=(-np.pi/2,np.pi/2)
    ),
    URDFLink(
      name="shoulder",
      origin_translation=[0.025, 0, 0.04],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-np.pi/2,np.pi/2)
    ),
    URDFLink(
      name="elbow",
      origin_translation=[0, 0, 0.12],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-np.pi/3,3*np.pi/4)
    ),
    URDFLink(
      name="wristroll",
      origin_translation=[0, 0, 0.09],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 1],
      bounds=(-np.pi/4,3*np.pi/4)
    ),
    URDFLink(
      name="wristbend",
      origin_translation=[0, 0, 0.03],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-np.pi/2,np.pi/2)
    ),
    URDFLink(
      name="gripperlink",
      origin_translation=[0, 0, 0.03],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 1],
      bounds=(-np.pi/2,np.pi/2)
    ),
])

print(arm_chain)

# Set to initial state with all joints at 0
current_pose = arm_chain.forward_kinematics( [0.]*len(arm_chain) )
print("FK position from joint states at 0's:")
print(current_pose)

# ===== IK test ( draw straight line) =====
cart_path =  zeros((50,3)) 
cart_path[:,0] = 0.1 # keep fixed distance away from origin in x axis
cart_path[:,1] = linspace(0.0,0.20,50) # z axis movement

# Creating figure 
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

joint_states = np.array([[0.]*7])
for target_xyz in cart_path:
    # Calculate the new joint state from the pose goal
    new_joints = update_pose(target_xyz, [0,90,0])
    joint_states = np.append(joint_states, np.array([[rad2degree(i) for i in new_joints]]), axis=0)
    print("New joints (deg):")
    print( ["{:.2f} ".format(rad2degree(i)) for i in new_joints])

    # Setting axis properties
    plt.cla()
    ax.set(xlim3d=(0,0.5), xlabel='X')
    ax.set(ylim3d=(0,0.5), ylabel='Y')
    ax.set(zlim3d=(0,0.5), zlabel='Z')
    
    # Update figure
    arm_chain.plot(new_joints, ax)
    plt.draw()
    plt.pause(0.1)
    
print("Close figure to continue")
plt.show()

dt = linspace(0,50,51)
for i in range(np.shape(joint_states)[1]):
    plt.plot(dt, joint_states[:,i], label = "Joint {:d}".format(i+1))
    
plt.title("Robot Joint Angles over Iteration")
plt.xlabel("Iteration")
plt.ylabel("Angle (deg)")
plt.legend()
plt.show()
    

# Overall it looks like it works! The jumpiness of the angles is a concern when choosing the new
# joint values. It's also using a different start pose (upright vs bend) but I think it's ok.
# Note: When I added the bounds it creates a smooth trajectory!