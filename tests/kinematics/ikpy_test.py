# Quick test to make sure the ikpy module works on the Windows PC. This is compared to the results
# with the old custom solver
import os
import sys
os.chdir('../pico/lib')
sys.path.append(os.getcwd())
from arm_utils.armTransforms import (Angle, eulerAnglesToRotMatrix, rad2degree,
                                     create_circular_trajectory, create_vertical_trajectory)
#from arm_control import robotarm
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import matplotlib.pyplot as plt


# miniarm_params = [{'theta':   0.000, 'alpha':    0.000, 'a':   0.000, 'd': 0.095},
#                   {'theta':-np.pi/2, 'alpha': -np.pi/2, 'a':   0.015, 'd': 0.000},
#                   {'theta':   0.000, 'alpha':    0.000, 'a':   0.120, 'd': 0.000},
#                   {'theta':   0.000, 'alpha': -np.pi/2, 'a':   0.000, 'd': 0.090},
#                   {'theta':   0.000, 'alpha':  np.pi/2, 'a':   0.000, 'd': 0.000},
#                   {'theta':   0.000, 'alpha': -np.pi/2, 'a':   0.000, 'd': 0.000},
#                   {'theta':   0.000, 'alpha':    0.000, 'a':   0.000, 'd': 0.030}]

def update_pose(xyz, rpy):
    """ calculate new joint state positions """
    rpy = eulerAnglesToRotMatrix(rpy[0], rpy[1], rpy[2]) # convert to rotation matrix
    new_joints = arm_chain.inverse_kinematics(
        target_position=xyz,
        target_orientation=rpy,
        orientation_mode='all',
    )        
    return new_joints
    
# Build robot chain
arm_chain1 = Chain(name='arm', links=[
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
arm_chain2 = Chain(name='miniarm', links=[
    OriginLink(),
    URDFLink(
      name="Base",
      origin_translation=[0, 0, 0.095], # d1
      origin_orientation=[0, 0,0],
      rotation=[0, 0, 1], # Z axis rotation
      bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
        name="shoulder",
        origin_translation=[0.015, 0, 0], # a2
        origin_orientation=[-np.pi/2, 0, 0], # a2 = -π/2
        rotation=[0,1,0],
        bounds=(-np.pi/2, np.pi/2) # limits for the shoulder joint
    ),
    URDFLink(
        name="elbow",
        origin_translation=[0.12, 0, 0], # a3
        origin_orientation=[0, 0, 0], # no offset for this joint
        rotation=[0, 1, 0], # rotation around Y for elbow
        bounds=(-np.pi/3, 3*np.pi/4) # limits for the elbow joint
    ),
    URDFLink(
        name="wristroll",
        origin_translation=[0.0, 0, 0.09], # d4
        origin_orientation=[-np.pi/2, 0, 0],  # α4 = -π/2
        rotation=[0, 0, 1], # rotation around Z for wrist roll
        bounds=(-np.pi/2, np.pi/2) # limits for the wrist roll joint
    ),
    URDFLink(
        name="wristpitch",
        origin_translation=[0, 0, 0], # no translation, only twist
        origin_orientation=[-np.pi/2, 0, 0],  # α5 = -π/2
        rotation=[0, 1, 0],
        bounds=(-np.pi/2, np.pi/2) # limits for the wrist bend joint
    ),
    URDFLink(
        name="gripperlink",
        origin_translation=[0.0, 0, 0.03], # d6 (gripper offset)
        origin_orientation=[-np.pi/2, 0, 0], # α6 = -π/2
        rotation=[0, 0, 1], # gripper rotation around Z
        bounds=(-np.pi/2, np.pi/2) # limits for the gripper joint
    ),

])
arm_chain3 = Chain(name='arm', links=[
    OriginLink(),
    URDFLink(
      name="Base",
      origin_translation=[0, 0, 0.095],
      origin_orientation=[0, 0,0],
      rotation=[0, 0, 1],
      bounds=(-np.pi/2,np.pi/2)
    ),
    URDFLink(
      name="shoulder",
      origin_translation=[0.015, 0, 0],
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

arm_chain = arm_chain3
print(arm_chain)

# Set to initial state with all joints at 0
current_pose = arm_chain.forward_kinematics( [0.]*len(arm_chain) )
print("FK position from joint states at 0's:")
print(current_pose)

# ===== IK test ( draw straight line) =====
cart_path =  np.zeros((50,3))
cart_path[:,0] = 0.1 # keep fixed distance away from origin in x axis
cart_path[:,1] = np.linspace(0.0,0.20,50) # z axis movement

# ===== IK test ( draw circular trajectory) =====
MAXSTEP = 101
circ_path = create_circular_trajectory([0.15, 0.000, 0.2], 0.01, MAXSTEP) # draw a circle

# Creating figure
USE_PLOT = True
fig = plt.figure()
ax = fig.add_subplot(projection="3d")


ee_path = circ_path
joint_states = np.array([[0.]*7])
for target_xyz in ee_path:
    # Calculate the new joint state from the pose goal
    new_joints = update_pose(target_xyz, [Angle(0,'deg'),Angle(90,'deg'),Angle(0,'deg')])
    #new_joints = update_pose(target_xyz, [Angle(0,'deg'),Angle(90,'deg'),Angle(0,'deg')])
    joint_states = np.append(joint_states, np.array([[rad2degree(i) for i in new_joints]]), axis=0)
    print("New joints (deg):")
    print( ["{:.2f} ".format(rad2degree(i)) for i in new_joints])

    if USE_PLOT:
        # Setting axis properties
        plt.cla()
        ax.set(xlim3d=(-0.1,0.2), xlabel='X')
        ax.set(ylim3d=(-0.1,0.2), ylabel='Y')
        ax.set(zlim3d=(0,0.2), zlabel='Z')

        # Update figure
        arm_chain.plot(new_joints, ax)
        plt.draw()
        plt.pause(0.1)
    
print("Close figure to continue")
plt.show() # Wait for close

dt = np.arange(0, len(joint_states))
for i in range(joint_states.shape[1]):
    plt.plot(dt, joint_states[:,i], label = "Joint {:d}".format(i+1))
    
plt.title("Robot Joint Angles over Iteration")
plt.xlabel("Iteration")
plt.ylabel("Angle (deg)")
plt.legend()
plt.show()
    

# Overall it looks like it works! The jumpiness of the angles is a concern when choosing the new
# joint values. It's also using a different start pose (upright vs bend) but I think it's ok.
# Note: When I added the bounds it creates a smooth trajectory!