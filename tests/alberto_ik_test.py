# This is a test of the kinematic solver package from Alberto Abarzua with an analytical method 
# having forward and inverse kinematic methods as well as robot poses and joint angles contained
# in specific objects.
#
# (https://github.com/alberto-abarzua/3d_printed_robot_arm). 
 
import arm_control.robotarm as robotarm
import arm_utils.armTransforms as util
from arm_utils.armTransforms import Angle, Config

robot = robotarm.RobotArm()
robot.a1z = 650
robot.a2x = 400
robot.a2z = 680
robot.a3z = 1100
robot.a4z = 230
robot.a4x = 766
robot.a5x = 345
robot.a6x = 244
       
       
def helper_direct_kinematics(robot, joints, pos, euler):
        """Helper method used to test direct_kinematics

        Args:
            joints (list[Angle]): angles for the robot joints in deg.
            pos (list[float]): x,y,z expected positions
            euler (list[float]): A,B,C expected euler angles in deg.
        """
        print("================== Helper forward Kinematic Test ====================")
        print("Passed joints: {}".format(joints))
        angles = util.angle_list(joints, "deg")
        config = robot.direct_kinematics(angles)
        print("Predicted position: {}".format(pos))
        print("Calculated position: {}".format(config.cords))
        print("Predicted euler angles: {}".format(euler))
        print("Calculated euler angle: {}".format([angle.deg for angle in config.euler_angles]))
        print("=====================================================================")

def helper_inverse_kinematics(robot, joints, pos, euler):
        """Helper method used to test inverse_kinematics (IK) from a configuration of positions and euler angles.

        Args:
            joints (list[Angle]): Expected angles for the robot (what IK should return)
            pos (list[float]): x,y,z positions of config
            euler (list[Angle]): A,B,C euler angles of config
        """
        print("================== Helper forward Kinematic Test ====================")
        print("Passed position: {}".format(pos))
        print("Passed euler angles: {}".format(euler))
        euler = util.angle_list(euler, "deg")
        angles = robot.inverse_kinematics(Config(pos, euler))
        angles = [angle.deg for angle in angles]
        print("Predicted angles: {}".format(joints))
        print("Calculated angles: {}".format(angles))
        print("=====================================================================")

def test_direct_kinematics(robot):
        helper_direct_kinematics(robot,
            [0, 0, 0, 0, 0, 0], [1755, 0, 2660], [0, 0, 0])
        helper_direct_kinematics(robot,
            [90, 0, 0, 0, 0, 0], [0, 1755, 2660], [0, 0, 90])
        helper_direct_kinematics(robot,
            [130, -60, 30, 60, -90, 60], [11.8, 314.7, 2740.3], [-176.3, -25.7, 23.9])
        helper_direct_kinematics(robot,
            [-46, 46, 46, 46, 46, 46], [962.3, -814.8, 810.6], [-132.0, 42.6, 89.3])

def test_inverse_kinematics(robot):
        helper_inverse_kinematics(robot,
            [0, 0, 0, 0, 0, 0], [1755, 0, 2660], [0, 0, 0])
        helper_inverse_kinematics(robot,
            [90, 0, 0, 0, 0, 0], [0, 1755, 2660], [0, 0, 90])
        helper_inverse_kinematics(robot,
            [38.5, 7.9, 25.5, 55.3, -49.2, -43.3], [1500, 1000, 2000], [0, 0, 0])
        helper_inverse_kinematics(robot,
            [124.3, -24.7, 43.6, -42.8, 65.6, 140.8], [-500, 1000, 2000], [50, 50, 50])
        helper_inverse_kinematics(robot,
            [-51.6, -7.9, -31.3, -51.5, 52.6, -72.7], [600, -1000, 3300], [250, 0, -90])

test_direct_kinematics(robot)
test_inverse_kinematics(robot)