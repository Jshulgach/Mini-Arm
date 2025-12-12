import os
import sys
#os.chdir('../lib')
#sys.path.append(os.getcwd())
from lib.arm_utils.armTransforms import *

import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import numpy as np
import matplotlib.pyplot as plt
 
# Puma dimensions (m), see RVC2 Fig. 7.4 for details
l1 = 0.672
l2 = -0.2337
l3 = 0.4318
l4 = 0.0203
l5 = 0.0837
l6 = 0.4318

robot = rtb.ERobot([
    rtb.Link(rtb.ET.tz(0.05) * rtb.ET.Rz(), name='base'),
    rtb.Link(rtb.ET.tx(0.025) * rtb.ET.tz(0.04) * rtb.ET.Ry(), name='shoulder', parent='base'),
    rtb.Link(rtb.ET.tz(0.12) * rtb.ET.Ry(), name='elbow', parent='shoulder'),
    rtb.Link(rtb.ET.tz(0.09) * rtb.ET.Rz(), name='wristroll', parent='elbow'),
    rtb.Link(rtb.ET.tz(0.03) * rtb.ET.Ry(), name='wristbend', parent='wristroll'),
    rtb.Link(rtb.ET.tz(0.03) * rtb.ET.Rz(), name='gripperlink', parent='wristbend'),
    rtb.Link(rtb.ET.tz(0.01), name='gripper', parent='gripperlink'),
])


print(robot)
#print(len(robot.elinks))

# ===== IK test ( draw straight line) =====
cart_path =  np.zeros((50,3))
cart_path[:,0] = 0.15
cart_path[:,2] = np.linspace(0.2,0.05,50)

last_sol = None
joint_states = np.array([[0.]*6])
for xyz in cart_path:
    Tep = SE3.Trans(xyz[0], xyz[1], xyz[2]) * SE3.RPY([0, 0, 0], order="xyz") * SE3.Ry(90, unit="deg")
    sol = robot.ikine_LM(Tep)         # solve IK
    if sol.success:
        new_joints = sol.q
        last_sol = sol.q
        joint_states = np.append(joint_states, np.array([new_joints]), axis=0)
        #joint_states = np.append(joint_states, np.array([[rad2degree(i) for i in new_joints]]), axis=0)
    else:
        print("Failed to find IK solution.")


#dt = np.linspace(0,50,51)
#for i in range(np.shape(joint_states)[1]):
#    plt.plot(dt, joint_states[:,i], label="Joint {:d}".format(i+1))
    
#plt.title("Robotics Toolbox: Robot Joint Angles over Iteration")
#plt.xlabel("Iteration")
#plt.ylabel("Angle (deg)")
#plt.legend()
#plt.show()


q_pickup = last_sol[0]
print(robot.fkine(q_pickup))    # FK shows that desired end-effector pose was achieved

qt = rtb.jtraj(robot.qr, q_pickup, 50)
robot.plot(qt.q, backend='pyplot', movie='panda1.gif')