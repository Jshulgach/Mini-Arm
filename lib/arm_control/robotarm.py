import gc
import time
import busio
import board
from adafruit_servokit import ServoKit
from arm_utils.armTransforms import *

__author__ = "Jonathan Shulgach"
__version__ = "0.5.9"

# Servo channel numbering on the PCA9685
BASE_SERVO           = 15
SHOULDER_SERVO       = 14
ELBOW_SERVO          = 13
WRIST_ROT_SERVO      = 12
WRIST_BEND_SERVO     = 11
GRIPPER_LINK_SERVO   = 10
GRIPPER_SERVO        = 9

def ServoIndex(key):
    """helper function to convert number to servo name
    """
    servo_dict = {'base':BASE_SERVO,               '1':BASE_SERVO,
                  'shoulder':SHOULDER_SERVO,       '2':SHOULDER_SERVO,
                  'elbow':ELBOW_SERVO,             '3':ELBOW_SERVO,
                  'wristrot':WRIST_ROT_SERVO,      '4':WRIST_ROT_SERVO,
                  'wristbend':WRIST_BEND_SERVO,    '5':WRIST_BEND_SERVO,
                  'gripperlink':GRIPPER_LINK_SERVO,'6':GRIPPER_LINK_SERVO,
                  'gripper':GRIPPER_SERVO,         '7':GRIPPER_SERVO,
                 }
    num = None
    if key in servo_dict.keys():
        num = servo_dict[key]
    return num

class RobotArm:
    """RobotArm class, used to create and calculate the kinematics of a robot arm.
    """

    def __init__(self, name='Robot', simulate=False, verbose=False):
        """Robot arm constructor, it creates the robot arm with it's physical parameters and 
        initializes with all the angles equal to zero.
        """
        self.name = name
        self.simulate = simulate
        self.verbose = verbose
        # Physical parameters
        # J1
        self.a1x = 0
        self.a1y = 0
        self.a1z = 0
        # J2
        self.a2x = 0
        self.a2y = 0
        self.a2z = 0
        # J3
        self.a3x = 0
        self.a3y = 0
        self.a3z = 0
        # J4
        self.a4x = 0
        self.a4y = 0
        self.a4z = 0
        # J5
        self.a5x = 0
        self.a5y = 0
        self.a5z = 0
        # J6
        self.a6x = 0
        self.a6z = 0
        self.a6y = 0

        # joints stepper ratios

        self.joint_ratios = []

        # constraints. (should be a tuple of Angle's )

        #           (Angle(min_val), Angle(max_val))
        default_min = -2 * np.pi
        default_max = 2 * np.pi

        self.j1_range = lambda x: x > default_min and x < default_max
        self.j1_range = lambda x: x > default_min and x < default_max
        self.j2_range = lambda x: x > default_min and x < default_max
        self.j3_range = lambda x: x > default_min and x < default_max
        self.j4_range = lambda x: x > default_min and x < default_max
        self.j5_range = lambda x: x > default_min and x < default_max
        self.j6_range = lambda x: x > default_min and x < default_max

        # Joints angles
        #self.pose = None
        self.angles = [Angle(0, "rad") for i in range(6)]
        self.config = Config([], [], 100)
        self.direct_kinematics()  # update self.config with initial values.

        # Create references to the servo hardware
        if not self.simulate:
            # necessary to create the custom busio.I2C object instead of the ServoKit default due to some dormant library issue 
            i2c = busio.I2C(board.GP1, board.GP0)
            self.servos = ServoKit(channels=16, i2c=i2c)

    @property
    def constraints(self):
        return [self.j1_range, self.j2_range, self.j3_range, self.j4_range, self.j5_range, self.j6_range]

    def logger(self, *argv):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def robotinfo(self):
        """ Helper function to display the state of the robot"""
        self.logger(
            "\n=========================================== Robot Info ===========================================" +
            "\nCurrent joint state: {}".format([angle.rad for angle in self.angles]) +
            "\nCurrent pose: \n{}".format(self.config.cords) +
            "\n==================================================================================================")

    def set_joints(self, new_angles, absolute=False):
        """Function that updates the joint state angles for the intrnal model as well as physical hardware

        :param new_angles: (list) a numeric list of the new joint angles for the robot
        """
        if self.verbose: self.logger("Set joint angles: {}".format(new_angles))
        for i, val in enumerate(new_angles):
            if self.verbose: self.logger("Changing joint:{} position to: {}".format(i+1, val))
            self.set_joint(i, val)
            
    def get_joints(self):
        return [i.deg for i in self.angles]
        
    def set_joint(self, idx, val):
        """ Function that moves the physical motor/servo to the specified angle
        
        :param idx: (int) joint index, indexing starts at zero
        :param val: (int) angle value to set the specified joint at
        """
        if not self.simulate:
            self.servos.servo[ServoInded(str(idx+1))].angle = val
        #time.sleep(0.05)  # Provide delay to give time for servos to update position
        
    def set_gripper(self, val):
        """ Function that updates only the gripper servo
        
        :param val: (int) angle value to set the specified joint at
        """
        if self.verbose: self.logger("Changing gripper position to: {}".format(val))
        self.set_joint(6, val)
    
    def get_gripper(self):
        return self.config.tool
        
    def set_pose(self, target_pos):
        """ Set the end-effector to a new position and update the robot angles accordingly
        
        :param target_pos: (list) numeric list of [x,y,z]  or [x, y, z, roll, pitch, yaw] coordinates
        """
        target_pos = [float(i) for i in target_pos] # Make sure values are not strings
        #euler = self.config.euler_angles
        #if len(target_pos)>3: euler = angle_list(target_pos[3:6], "deg")
        
        self.angles = self.inverse_kinematics(Config(target_pos[:3], angle_list(target_pos[3:6], "deg")))
        if self.verbose: self.logger("Calculated angles: {}".format([angle.deg for angle in self.angles]))
        #self.set_joints([i.deg for i in self.angles])
        gc.collect() # garbage collector: free up memory
        
    def handle_delta(self, delta_pos):
        """ Adjust the end effector position to a new relative position using the delta input. 
            Expecting small inputs (0.01 increments)
        
        :param target_pos: (list) numeric list of [x,y,z] or [x,y,z, roll, pitch, yaw] coordinates
        """
        delta_pos = [float(i) for i in delta_pos] # Make sure values are not strings
        target_xyz = [a + b for a, b in zip(self.config.cords[:3], delta_pos[:3])]
        target_euler = [a.deg + b for a, b in zip(self.config.euler_angles[:3], delta_pos[3:6])]
        return target_xyz + target_euler
 
    def direct_kinematics(self, angles=None):
        """Direct Kinematics function,takes a configuration of angles for all the robots joints and 
        calculates the current position of the Arms tool and it's euler angles. If the list
        of angles is not provided direct_kinematics will use the robots current angles.

        Args:
            angles (Angle, optional): list of Angles to use as robot config. Defaults to None.

        Returns:
            Config: returns the current position of the tool and the current euler angles
            as a Config.
        """
        # -----------------------------------------------------------------------------------
        # -----------------------------------------------------------------------------------
        # FORWARD KINEMATCIS
        # -----------------------------------------------------------------------------------
        # -----------------------------------------------------------------------------------

        if (angles == None):
            angles = self.angles
        J1, J2, J3, J4, J5, J6 = angles
        # BASE --> J1
        R1 = zmatrix(J1)
        D1 = np.array([
            [self.a1x],
            [self.a1y],
            [self.a1z]
        ])
        T1 = tmatrix(R1, D1)
        # J1 -->  J2
        R2 = ymatrix(J2)
        D2 = np.array([
            [self.a2x],
            [self.a2y],
            [self.a2z]
        ])
        T2 = tmatrix(R2, D2)
        # J2 -->  J3
        R3 = ymatrix(J3)
        D3 = np.array([
            [self.a3x],
            [self.a3y],
            [self.a3z]
        ])
        T3 = tmatrix(R3, D3)
        # J3 -->  J4

        R4 = xmatrix(J4)
        D4 = np.array([
            [self.a4x],
            [self.a4y],
            [self.a4z]
        ])
        T4 = tmatrix(R4, D4)
        # J4 -->  J5
        R5 = ymatrix(J5)
        D5 = np.array([
            [self.a5x],
            [self.a5y],
            [self.a5z]
        ])
        T5 = tmatrix(R5, D5)
        # J5 -->  J6
        R6 = xmatrix(J6)
        D6 = np.array([
            [self.a6x],
            [self.a6y],
            [self.a6z]
        ])
        T6 = tmatrix(R6, D6)
        # Base--> TCP
        temp1 = np.dot(np.dot(np.dot(np.dot(np.dot(T1, T2), T3), T4), T5), T6)
        position = np.dot(temp1, np.array([[0], [0], [0], [1]]))
        #position = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ np.array([[0], [0], [0], [1]])

        rotation = np.dot(np.dot(np.dot(np.dot(np.dot(R1, R2), R3), R4), R5), R6)
        #rotation = R1 @ R2 @ R3 @ R4 @ R5 @ R6
        euler_angles = rotationMatrixToEulerAngles(rotation)
        pos = list(position[:3, 0])

        self.config.cords = pos[0:3]
        self.config.euler_angles = euler_angles
        #self.pose.cords = pos[0:3]
        #self.pose.euler_angles = euler_angles
        return self.config
        
    def helper_direct_kinematics(self, joints, pos, euler):
        """Helper method used to test direct_kinematics

        Args:
            joints (list[Angle]): angles for the robot joints in deg.
            pos (list[float]): x,y,z expected positions
            euler (list[float]): A,B,C expected euler angles in deg.
        """
        self.logger("================== Helper forward Kinematic Test ====================")
        self.logger("Passed joints: {}".format(joints))
        angles = angle_list(joints, "deg")
        config = self.direct_kinematics(angles)
        self.logger("Predicted position: {}".format(pos))
        self.logger("Calculated position: {}".format(config.cords))
        self.logger("Predicted euler angles: {}".format(euler))
        self.logger("Calculated euler angle: {}".format([angle.deg for angle in config.euler_angles]))
        self.logger("=====================================================================")

    def inverse_kinematics(self, config=None):
        """Safe version of inverse kinematics, calls not_safe_IK that does all the calculations.

        Args:
            config (Config), optional): configuration of the robot to reach. Defaults to None.

        Raises:
            OutOfBoundsError: if the config is not achievable an exception is raised.

        Returns:
            list[Angle]: list of Angles the robot arm should have to reach config.
        """
        # -----------------------------------------------------------------------------------
        # -----------------------------------------------------------------------------------
        # INVERSE KINEMATICS
        # -----------------------------------------------------------------------------------
        # -----------------------------------------------------------------------------------
    
        res = self.not_safe_IK(config)
        if res is None:
            if self.verbose: self.logger("Position {} out of range".format([config.cords],[i.deg for i in config.euler_angles]))
            if self.verbose: self.logger("Using last known position {}".format([self.config.cords],[i.deg for i in self.config.euler_angles]))
            res = self.angles
        else:
            self.update_current_pose(config)
        return res

    def update_current_pose(self, config):
        """ Helper function to update the current robot pose """
        self.config.cords = config.cords
        self.config.euler_angels = config.euler_angles
        self.config.tool = config.tool

    def not_safe_IK(self, config=None):
        """OLD METHOD Gets the angles that the robots joints should have so that the tool is in the position and
        euler angles given by config. If config is None the current config of the robot arm is used.

        Args:
            config (Config), optional): configuration of the robot to reach. Defaults to None.

        Returns:
            list[Angle]: list of Angles the robot arm should have to reach config.
        """
        prev = self.angles[:]  # Current angles of the robot (used to choose the closest angles to achieve config)
        if (config == None):
            config = self.config
        xyz, euler_angles = config.cords, config.euler_angles
        x, y, z = xyz
        A, B, C = euler_angles

        TCP = np.array([
            [x],
            [y],
            [z]
        ])
        xdirection = np.dot(eulerAnglesToRotMatrix(A, B, C), np.array([[1], [0], [0]]))
        WP = TCP - self.a6x * xdirection
        # Finding J1,J2,J3

        J1 = Angle(np.arctan2(WP[1, 0], WP[0, 0]), "rad")
        if (WP[0, 0] == 0 and WP[1, 0] == 0):
            # Singularity, if Wx = Wy =0 dejar J1 como pos actual.
            J1 = self.angles[0]
        WPxy = np.sqrt(WP[0, 0] ** 2 + WP[1, 0] ** 2)
        L = WPxy - self.a2x
        H = WP[2, 0] - self.a1z - self.a2z
        P = np.sqrt(H ** 2 + L ** 2)
        b4x = np.sqrt(self.a4z ** 2 + (self.a4x + self.a5x) ** 2)
        
        if (P <= self.a3z + b4x) and abs(self.a3z - b4x) < P:
            alfa = np.arctan2(H, L)
            cosbeta = (P ** 2 + self.a3z ** 2 - b4x ** 2) / (2 * P * self.a3z)
            beta = np.arctan2(np.sqrt(1 - cosbeta ** 2), cosbeta)
            cosgamma = (self.a3z ** 2 + b4x ** 2 - P ** 2) / (2 * self.a3z * b4x)
            gamma = np.arctan2(np.sqrt(1 - cosgamma ** 2), cosgamma)
            lamb2 = np.arctan2(self.a3x, self.a3z)
            delta = np.arctan2(self.a4x + self.a5x, self.a4z)
            J2 = Angle(np.pi / 2.0 - alfa - beta, "rad")
            J3 = Angle(np.pi - gamma - delta, "rad")
            # Finding Wrist Orientation
            R1 = zmatrix(J1)
            R2 = ymatrix(J2)
            R3 = ymatrix(J3)
            Rarm = np.dot(np.dot(R1, R2), R3)
            Rarmt = Rarm.transpose()
            R = eulerAnglesToRotMatrix(A, B, C)
            Rwrist = np.dot(Rarmt, R)
            # Finding J4
            J5 = Angle(np.arctan2(
                np.sqrt(1 - Rwrist[0, 0] ** 2), Rwrist[0, 0]), "rad")
            if J5.rad == 0:  # Singularity
                J4 = self.angles[3]  # keep the current angle of J4.
                J6 = Angle(np.arctan2(
                    Rwrist[2, 1], Rwrist[2, 2]), "rad").sub(J4)
            else:
                J4_1 = Angle(np.arctan2(Rwrist[1, 0], -Rwrist[2, 0]), "rad")
                J4_2 = Angle(-np.arctan2(Rwrist[1, 0], Rwrist[2, 0]), "rad")

                J6_1 = Angle(np.arctan2(Rwrist[0, 1], Rwrist[0, 2]), "rad")
                J6_2 = Angle(-np.arctan2(Rwrist[0, 1], -Rwrist[0, 2]), "rad")
                if (abs(prev[3].rad - J4_1.rad) > abs(prev[3].rad - J4_2.rad)):
                    J4 = J4_2
                    J6 = J6_2
                    J5 = Angle(np.arctan2(
                        -np.sqrt(1 - Rwrist[0, 0] ** 2), Rwrist[0, 0]), "rad")
                else:
                    J4 = J4_1
                    J6 = J6_1
            calculated_joints = [J1, J2, J3, J4, J5, J6]
            return nearest_to_prev([J1, J2, J3, J4, J5, J6], prev)
        else:
            #return [Angle(0, "rad") for angle in range(0,6)]
            return None
            
    def helper_inverse_kinematics(self, joints, pos, euler):
        """Helper method used to test inverse_kinematics (IK) from a configuration of positions and euler angles.

        Args:
            joints (list[Angle]): Expected angles for the robot (what IK should return)
            pos (list[float]): x,y,z positions of config
            euler (list[Angle]): A,B,C euler angles of config
        """
        self.logger("================== Helper Inverse Kinematic Test ====================")
        self.logger("Passed position: {}".format(pos))
        self.logger("Passed euler angles: {}".format(euler))
        angles = self.inverse_kinematics(Config(pos, angle_list(euler, "deg")))
        self.logger("Predicted angles: {}".format(joints))
        self.logger("Calculated angles: {}".format([angle.deg for angle in angles]))
        self.logger("=====================================================================")

