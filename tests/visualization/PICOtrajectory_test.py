# This is a test of the servo capabilities from the Adafruit ServoKit and being able to update the servos at
# a quick enough rate so using the IK solver, draw a circle in cartesian space as smooth as possible

# So far the fastest average rate I've seen running pose commands to the robot: 26Hz 
#
#
import os
import sys
os.chdir('../pico/lib')
sys.path.append(os.getcwd())

import time
#import ulab.numpy as np
import numpy as np
#import arm_control.robotarm as robotarm
import matplotlib.pyplot as plt

from arm_utils.robot_models import miniarm_params
from arm_utils.armTransforms import create_circular_trajectory
from arm_control.robotarm import RobotArm

# robot = robotarm.RobotArm(simulate_hardware=False)
# robot.a1z = 50
# robot.a2x = 0
# robot.a2z = 40
# robot.ax2 = 10
# robot.a3z = 120
# robot.a4z = 0
# robot.a4x = 90
# robot.a5x = 30
# robot.a6x = 50
# robot.direct_kinematics()  # update self.config with new values.
# robot.robotinfo()

robot = RobotArm(simulate_hardware=True,
                 dh_params=miniarm_params,
                 speed_control=False,
                 verbose=False)

#def create_circular_trajectory(center, radius=10):
#    theta = np.linspace(0, 2*np.pi, 201)
#    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
#    return center + radius*temp
        
# ===== IK test ( draw circular trajectory) =====
MAXSTEP = 101
circ_path = create_circular_trajectory([0.15, 0.000, 0.2], 0.01, MAXSTEP) # draw a circle
#points = create_circular_trajectory([150, 0, 260], 40) # draw a circle
euler = np.array([0.00,0.00,0.00]) # keep same orientation for all points

prev_t = 0

ee_path = circ_path
joint_states = np.array([[0.]*6])

for target_xyz in ee_path:
    pose = np.concatenate((target_xyz, euler), axis=0)
    print("pose to send: {}".format(pose))
    robot.set_pose(pose)
    new_joints = robot.get_joints()
    joint_states = np.append(joint_states, np.array([[i for i in new_joints]]), axis=0)

    #robot.robotinfo()

    #print("{}".format(1/(time.monotonic()-prev_t)))
    #prev_t = time.monotonic()
    #time.sleep(0.025)

dt = np.arange(0, len(joint_states))
for i in range(joint_states.shape[1]):
    plt.plot(dt, joint_states[:, i], label="Joint {:d}".format(i + 1))

plt.title("Robot Joint Angles over Iteration")
plt.xlabel("Iteration")
plt.ylabel("Angle (deg)")
plt.legend()
plt.show()
