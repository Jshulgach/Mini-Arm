import roboticstoolbox as rtb
from roboticstoolbox import ERobot, Link
from spatialmath import SE3
import numpy as np
import matplotlib.pyplot as plt

mesh_list = ['3d_files/mesh/base_link.STL',
             '3d_files/mesh/link_1.STL',
             '3d_files/mesh/link_2.STL',
             '3d_files/mesh/link_3.STL',
             '3d_files/mesh/link_4.STL',
             '3d_files/mesh/link_5.STL',
             '3d_files/mesh/gripper.STL',]

# Define your custom robot using ERobot
robot = ERobot([
    Link(rtb.ET.tz(0.05) * rtb.ET.Rz(), name='Base', meshfile=mesh_list[0]),
    Link(rtb.ET.tx(0.025) * rtb.ET.tz(0.04) * rtb.ET.Ry(), name='shoulder', parent='Base', meshfile=mesh_list[1]),
    Link(rtb.ET.tz(0.12) * rtb.ET.Ry(), name='elbow', parent='shoulder', meshfile=mesh_list[2]),
    Link(rtb.ET.tz(0.09) * rtb.ET.Rz(), name='wristroll', parent='elbow',   meshfile=mesh_list[3]),
    Link(rtb.ET.tz(0.03) * rtb.ET.Ry(), name='wristbend', parent='wristroll', meshfile=mesh_list[4]),
    Link(rtb.ET.tz(0.03) * rtb.ET.Rz(), name='gripperlink', parent='wristbend', meshfile=mesh_list[5]),
    Link(rtb.ET.tz(0.01), name='gripper', parent='gripperlink', meshfile=mesh_list[6]),
])

print(robot)

# Define a rest position (e.g., all zeros)
robot.qr = np.zeros(robot.n)  # "qr" is now explicitly defined

# Desired end-effector pose
Tep = SE3.Trans(0.15, 0, 0.2) * SE3.OA([0, 1, 0], [0, 0, -1])

# Solve inverse kinematics
sol = robot.ikine_LM(Tep)  # Solve IK using Levenberg-Marquardt method
if sol.success:
    q_pickup = sol.q  # Joint angles to achieve the desired pose
    print("Inverse Kinematics Solution:", q_pickup)

    # Forward Kinematics for verification
    print("End-effector pose:", robot.fkine(q_pickup))
else:
    print("IK solution failed")

# Generate a trajectory from the rest position to the target position
qt = rtb.jtraj(robot.qr, q_pickup, 50)  # 50 points in trajectory

# Visualize the robot motion
robot.plot(qt.q, backend='pyplot', movie='custom_robot_demo.gif')

# Optional: Plot joint angles over time
dt = np.linspace(0, 50, len(qt.q))
plt.figure()
for i in range(qt.q.shape[1]):
    plt.plot(dt, qt.q[:, i], label=f"Joint {i+1}")
plt.title("Joint Angles Over Trajectory")
plt.xlabel("Time Step")
plt.ylabel("Angle (rad)")
plt.legend()
plt.show()
