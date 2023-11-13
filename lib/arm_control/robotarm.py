
import time
import busio
import board
from adafruit_servokit import ServoKit
from arm_utils.armTransforms import *

__author__ = "Jonathan Shulgach"
__version__ = "1.0.1"

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
    servo_dict = {'base':BASE_SERVO,               1:BASE_SERVO,
                  'shoulder':SHOULDER_SERVO,       2:SHOULDER_SERVO,
                  'elbow':ELBOW_SERVO,             3:ELBOW_SERVO,
                  'wristrot':WRIST_ROT_SERVO,      4:WRIST_ROT_SERVO,
                  'wristbend':WRIST_BEND_SERVO,    5:WRIST_BEND_SERVO,
                  'gripperlink':GRIPPER_LINK_SERVO,6:GRIPPER_LINK_SERVO,
                  'gripper':GRIPPER_SERVO,         7:GRIPPER_SERVO,
                 }
    num = None
    if key in servo_dict.keys():
        num = servo_dict[key]
    return num

class RobotArm(object):
    """RobotArm class, used to create and calculate the kinematics of a robot arm.
    """

    def __init__(self, name='Robot', simulate_hardware=False, verbose=False):
        """Robot arm constructor, it creates the robot arm with it's physical parameters and 
        initializes with all the angles equal to zero.
        """
        self.name = name
        self.simulate_hardware = simulate_hardware
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
        
        # DH parameters, list of dicts( 'theta', 'alpha', 'a', 'd')
        self.dh = None

        # joints constraints
        default_min = -np.pi/2
        default_max = np.pi/2
        self.flip_direction = [False, False, True, True, True, True, False]
        #self.joint_offsets = [0, 0, 0, 0, 0, 0, 0] 
        self.joint_offsets = [90, 90, 90, 90, 90, 90, 0] 
        self.joint_limits = [lambda x: x > default_min and x < default_max for x in range(0,6)]

        # Joints angles
        self.angles = [Angle(0.00, "rad") for i in range(6)] # Used to tracking the joint state
        self.config = Config([], [], 100) # Used for tracking the end effector pose
        self.reference_pos = None
        #self.NEW_forward_kinematics()
        #self.direct_kinematics()  # update self.config with initial values.

        # Create references to the servo hardware
        if not self.simulate_hardware:
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
        """ Helper function to display the state of the robot. The outpout will look like below
        
        Current joint state: [90, 90, 90, 90, 90, 90]
        Current pose:   cords: [x: 185.00000, y: 0.00000, z: 215.00000]
          angles: [Roll: 0.00000, Pitch: -0.00000, Yaw:0.00000]
          tool: 100
        """
        self.logger("\nCurrent joint state: {}\nCurrent pose: {}".format(self.get_joints(),self.config))
        self.logger("Calculated angles: {}".format(self.get_joints()))
            
    def set_joints(self, new_angles, absolute=False):
        """Function that updates the joint state angles for the internal model as well as physical hardware

        Parameters:
        ------------
        new_angles    : (list) a numeric list of the new joint angles for the robot  
        """
        self.angles = angle_list(new_angles[0:6], "deg")
        if self.verbose: self.logger("Set joint angles: {}".format(new_angles))
        for i, val in enumerate(new_angles):
            #if self.verbose: self.logger("Changing joint:{} position to: {}".format(i+1, val + self.joint_offsets[i]))
            self.set_joint(i, val)
                        
    def get_joints(self):
        #return [angle.deg + self.joint_offsets[i] for i, angle in enumerate(self.angles)]
        return [angle.deg for i, angle in enumerate(self.angles)]
        
    def set_joint(self, i, val):
        """ Function that moves the physical motor/servo to the specified angle
        
        :param idx: (int) joint index, indexing starts at zero
        :param val: (int) angle value to set the specified joint at
        """
        
        # Check that the proper motor indexing is happenning
        if i > 6:
            self.logger("Warning - Motor indexing starts at 0")
            return
        
        if not self.simulate_hardware:
            # Some actuators might be mounted in a way that the rotation direction needs to be reversed
            if self.flip_direction[i]: val = -val
            
            # Check that the angle range is within limits of the actuator
            if self.check_within_range(i, val + self.joint_offsets[i]):
                if self.verbose: self.logger("Changing joint:{} position to: {}".format(i+1, val+ self.joint_offsets[i]))
                
                # Set the new position to the actuator. ServoIndex dictionary is only necessary for custom
                # channel mapping from the servo motor shield
                self.servos.servo[ServoIndex(i+1)].angle = val + self.joint_offsets[i]
                
                # Update the internal joint state
                angles = self.get_joints()  
                angles[i] = val
                self.forward_kinematics(angle_list(angles, "deg")) # Update the new position of the gripper
            else:
                if self.verbose: self.logger("Warning. Servo value passed '{}' not within range. Cancelling new joint position".format(val + self.joint_offsets[i]))
                
    def validate_joint_angles(self, angles):
        """ Helper function that checks the sign for each of the joints to check it's between 0-180 
        and corrects them if they scale over
        
        Parameters:
        -----------
        angles     : (list) Old list of Angle elements
        
        Return:
        -----------
        angles     : (list) New list of Angle elements
        """
        for i, angle in enumerate(angles):
            angles[i] = enforcePositiveServoRange(angle)
            
        return angles
            
    def check_within_range(self, i, val):
        """ Helper function that determines whether the angle is within the range of the specified joint.
        For now it's just going to check that the angle is between 0-180
        
        :param idx: (int) joint index, indexing starts at zero
        :param val: (int) angle value to set the specified joint at

        :return: (bool)
        """
        if val >=0 and val <= 180:
            return True
        else: 
            return False

    def set_gripper(self, val):
        """ Function that updates only the gripper servo
        
        :param val: (int) angle value to set the specified joint at
        """
        if self.verbose: self.logger("Changing gripper position to: {}".format(val))
        self.servos.servo[ServoIndex('gripper')].angle = val
        #self.set_joint(6, val)
    
    def get_gripper(self):
        return self.config.tool
                
    def set_pose(self, target_pos):
        """ Set the end-effector to a new position and update the robot angles accordingly
        
        :param target_pos: (list) numeric list of [x,y,z]  or [x, y, z, roll, pitch, yaw] coordinates
        """
        target_pos = [float(i) for i in target_pos] # Make sure values are not strings
        #try:
        if True:
            if len(target_pos)>3:
                config_msg = Config(target_pos[:3], angle_list(target_pos[3:6], "deg"))
            else:
                angles = [x.deg for x in self.config.euler_angles]
                config_msg = Config(target_pos[:3], angle_list(angles, "deg"))
                
            self.angles = self.inverse_kinematics(config_msg)
            self.update_current_pose(config_msg)

            if self.verbose: self.logger("Calculated angles: {}".format(self.get_joints()))
            self.set_joints([i.deg for i in self.angles])
            if len(target_pos) > 6: self.set_gripper(target_pos[6])
        #except:
        #    if self.verbose: self.logger("Error in setting pose. Check values are of float type and IK solver has no issues")
                
    def handle_delta(self, delta_pos):
        """ Adjust the end effector position to a new relative position using the delta input. 
            Expecting small inputs (0.01 increments)
        
        :param target_pos: (list) numeric list of [x,y,z] or [x,y,z, roll, pitch, yaw] coordinates
        """
        delta_pos = [float(i) for i in delta_pos] # Make sure values are not strings
        target_xyz = [a + b for a, b in zip(self.config.cords[:3], delta_pos[:3])]
        target_euler = [a.deg + b for a, b in zip(self.config.euler_angles[:3], delta_pos[3:6])]
        
        target_pos = target_xyz + target_euler
        if len(delta_pos) > 6:
            target_pos.append(delta_pos[6])
        return  target_pos
        
    def handle_posture_delta(self, delta_pos):
        
        try:
            # Get the static reference position on the first request
            if not self.reference_pos:
                self.reference_pos = self.config.cords[:3] + self.config.euler_angles[:3]
            
            delta_pos = [float(i) for i in delta_pos] # Make sure values are not strings
            target_xyz = [a + b for a, b in zip(self.reference_pos[:3], delta_pos[:3])]
            target_euler = [a.deg + b for a, b in zip(self.reference_pos[3:], delta_pos[3:6])]
        
            target_pos = target_xyz + target_euler
        except:
            if self.verbose: self.logger("Error in setting pose. Check values are of float type")  
            target_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]            

        if len(delta_pos) > 6:
            target_pos.append(delta_pos[6])
        return  target_pos
 
    def update_current_pose(self, config):
        """ Helper function to update the current robot pose """
        self.config.cords = config.cords
        self.config.euler_angels = config.euler_angles
        self.config.tool = config.tool

    # -----------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------
    # FORWARD KINEMATICS
    # -----------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------

    def forward_kinematics(self, angles=None):
        """ 2nd version of forward kinematics calculator using DH parameters instead
        Takes a configuration of angles for all the robots joints and 
        calculates the current position of the Arms tool and it's euler angles. If the list
        of angles is not provided direct_kinematics will use the robots current angles.

        Args:
            angles (Angle, optional): list of Angles to use as robot config. Defaults to None.

        Returns:
            Config: returns the current position of the tool and the current euler angles
            as a Config.
        
        """
        if (angles == None):
            angles = self.angles

        T01, T12, T23, T34, T45, T56, T6g = self.get_transformations(angles)
        T0g = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(T01, T12), T23), T34), T45), T56), T6g) # dot product for all of them!
        self.config.cords = list(T0g[:3, 3])
        
        # need to adjust for tool frame
        R0g = np.dot(T0g[:3,:3], np.dot( ymatrix( Angle(np.pi/2,"rad")), zmatrix( Angle(-np.pi,"rad")) ) ) # dot product!!!
        self.config.euler_angles = rotationMatrixToEulerAngles(R0g)

        return self.config

    def helper_forward_kinematics(self, joints, pos, euler):
        """Helper method used to test direct_kinematics

        Args:
            joints (list[Angle]): angles for the robot joints in deg.
            pos (list[float]): x,y,z expected positions
            euler (list[float]): A,B,C expected euler angles in deg.
        """
        self.logger("================== Helper forward Kinematic Test ====================")
        self.logger("Passed joints: {}".format(joints))
        angles = angle_list(joints, "rad")
        config = self.forward_kinematics(angles)
        self.logger("Predicted position: {}".format(pos))
        self.logger("Calculated position: {}".format(config.cords))
        self.logger("Predicted euler angles: {}".format(euler))
        self.logger("Calculated euler angle: {}".format([angle.rad for angle in config.euler_angles]))
        self.logger("=====================================================================")

    def get_transformations(self, angles=None):
        """ Helper function that outputs all the homogenous transformations in the kinematic chain
        """
        if (angles == None):
            angles = self.angles
        J1, J2, J3, J4, J5, J6 = angles

        T01 = TransformationMatrixDH(J1.rad + self.dh[0]['theta'], self.dh[0]['alpha'], self.dh[0]['a'], self.dh[0]['d'])
        T12 = TransformationMatrixDH(J2.rad + self.dh[1]['theta'], self.dh[1]['alpha'], self.dh[1]['a'], self.dh[1]['d'])
        T23 = TransformationMatrixDH(J3.rad + self.dh[2]['theta'], self.dh[2]['alpha'], self.dh[2]['a'], self.dh[2]['d'])
        T34 = TransformationMatrixDH(J4.rad + self.dh[3]['theta'], self.dh[3]['alpha'], self.dh[3]['a'], self.dh[3]['d'])
        T45 = TransformationMatrixDH(J5.rad + self.dh[4]['theta'], self.dh[4]['alpha'], self.dh[4]['a'], self.dh[4]['d'])
        T56 = TransformationMatrixDH(J6.rad + self.dh[5]['theta'], self.dh[5]['alpha'], self.dh[5]['a'], self.dh[5]['d'])
        T6g = TransformationMatrixDH(self.dh[6]['theta'], self.dh[6]['alpha'], self.dh[6]['a'], self.dh[6]['d'])
        return T01, T12, T23, T34, T45, T56, T6g

    def OLD_direct_kinematics(self, angles=None):
        """Direct Kinematics function,takes a configuration of angles for all the robots joints and 
        calculates the current position of the Arms tool and it's euler angles. If the list
        of angles is not provided direct_kinematics will use the robots current angles.

        Args:
            angles (Angle, optional): list of Angles to use as robot config. Defaults to None.

        Returns:
            Config: returns the current position of the tool and the current euler angles
            as a Config.
        """
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
        T1 = TransformationMatrix(R1, D1)
        #print(T1)
        # J1 -->  J2
        R2 = ymatrix(J2)
        #R2 = xmatrix(J2)
        D2 = np.array([
            [self.a2x],
            [self.a2y],
            [self.a2z]
        ])
        T2 = TransformationMatrix(R2, D2)
        #print(T2)
        # J2 -->  J3
        R3 = ymatrix(J3)
        D3 = np.array([
            [self.a3x],
            [self.a3y],
            [self.a3z]
        ])
        T3 = TransformationMatrix(R3, D3)
        #print(T3)
        # J3 -->  J4
        R4 = xmatrix(J4)
        D4 = np.array([
            [self.a4x],
            [self.a4y],
            [self.a4z]
        ])
        T4 = TransformationMatrix(R4, D4)
        #print(T4)
        # J4 -->  J5
        R5 = ymatrix(J5)
        D5 = np.array([
            [self.a5x],
            [self.a5y],
            [self.a5z]
        ])
        T5 = TransformationMatrix(R5, D5)
        #print(T5)
        # J5 -->  J6
        R6 = xmatrix(J6)
        D6 = np.array([
            [self.a6x],
            [self.a6y],
            [self.a6z]
        ])
        T6 = TransformationMatrix(R6, D6)
        #print(T6)

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
        return self.config
        
    # -----------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------
    # INVERSE KINEMATICS
    # -----------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------
            
    def inverse_kinematics(self, config=None):
        """NEW Safe version of inverse kinematics, calls not_safe_IK that does all the calculations.

        Args:
            config (Config), optional): configuration of the robot to reach. Defaults to None.

        Returns:
            list[Angle]: list of Angles the robot arm should have to reach config.
        """
        joint_angles = self.not_safe_IK(config)
        if joint_angles is None:
            if self.verbose: self.logger("Position {} beyond physical range. Restoring previous pose".format([config.cords],[i.deg for i in config.euler_angles]))
            return self.angles

        # Implement check to make sure that angles don't flip 180 degrees 
        #print("before: {}".format(self.get_joints()))
        #joint_angles = self.validate_joint_angles(joint_angles)
        #print("after: {}".format(self.get_joints()))
         
        # Check all joint angles are within limits
        #if not all([self.joint_limits[i](val.rad) for i,val in enumerate(joint_angles)]):
        #    self.logger("A joint is hitting its limit. Restoring previous pose".format([config.cords],[i.deg for i in config.euler_angles]))
        #    return self.angles        

        return joint_angles

    def not_safe_IK(self, config):
    
        #try:
        if True:
            if (config == None):
                config = self.config
                
            xyz, euler_angles = config.cords, config.euler_angles
            A, B, C = euler_angles
        
            # Find gipper transformation with respect to base
            Rgu = np.dot(zmatrix(Angle(np.pi, "rad")), ymatrix(Angle(-np.pi/2, "rad")))    
            R0u = eulerAnglesToRotMatrix(A, B, C)
            R0g = np.dot(R0u, Rgu.transpose())

            # Find wrist center urdf transformation 
            wrist_center = self.get_wrist_center(xyz, R0g, self.dh[6]['d'])

            # Find joints 1-3
            J1, J2, J3 = self.get_first_three_angles(wrist_center)

            # Find wrist transformation with respect to base given new angles
            angles = self.angles
            prev = angles
            angles[3:6] = J1, J2, J3
            T01, T12, T23, _ , _ , _ , _ = self.get_transformations(angles)     

            # Find joints 4-6            
            T03 = np.dot(np.dot(T01, T12), T23)    
            R36 = np.dot(T03[:3,:3].transpose(), R0g)
            J4, J5, J6 = self.get_last_three_angles(R36)
        
            return nearest_to_prev([J1, J2, J3, J4, J5, J6], prev)
        #except:
        #    return None

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
        angles = self.inverse_kinematics(Config(pos, angle_list(euler, "rad")))
        self.logger("Predicted angles: {}".format(joints))
        self.logger("Calculated angles: {}".format([angle.rad for angle in angles]))
        #self.logger("Calculated angles: {}".format([angle.deg for angle in angles]))
        self.logger("=====================================================================")
        
    def get_first_three_angles(self, wrist_center):
    
        x, y, z  = wrist_center
        l = get_hypotenuse(self.dh[3]['d'], -self.dh[3]['a'])
        phi = np.arctan2(self.dh[3]['d'], -self.dh[3]['a'])
        x_prime = get_hypotenuse(x, y)
        mx = x_prime -  self.dh[1]['a']
        my = z - self.dh[0]['d']
        m = get_hypotenuse(mx, my)
        alpha = np.arctan2(my, mx)
  
        gamma = get_cosine_law_angle(l, self.dh[2]['a'], m)
        beta = get_cosine_law_angle(m, self.dh[2]['a'], l)
  
        q1 = np.arctan2(y, x)
        q2 = np.pi/2 - beta - alpha 
        q3 = -(gamma - phi)
     
        return Angle(q1, "rad"), Angle(q2, "rad"), Angle(q3, "rad")
        
    def get_last_three_angles(self, R):

        sin_q5 = np.sqrt(R[0, 2]**2 + R[2, 2]**2) 
  
        q4 = np.arctan2(R[2, 2], -R[0, 2])
        q5 = np.arctan2(sin_q5, R[1, 2])
        q6 = np.arctan2(-R[1, 1], R[1, 0])
        
        # because this model has some singularities, we may consider
        if( np.sin(q5) < 0 ):
            q4 = np.arctan2(-R[2, 2], R[0, 2])
            q6 = np.arctan2(R[1, 1], -R[1, 0])
    
        return Angle(q4, "rad"), Angle(q5, "rad"), Angle(q6, "rad")
    
    def get_wrist_center(self, gripper_point, R0g, dg = 0.303):

        xu, yu, zu = gripper_point 
    
        nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
        xw = xu - dg * nx
        yw = yu - dg * ny
        zw = zu - dg * nz 
        
        return xw, yw, zw
  
    def OLD_inverse_kinematics(self, config=None):
        """Safe version of inverse kinematics, calls not_safe_IK that does all the calculations.

        Args:
            config (Config), optional): configuration of the robot to reach. Defaults to None.

        Returns:
            list[Angle]: list of Angles the robot arm should have to reach config.
        """
        joint_angles = self.not_safe_IK(config)
        if joint_angles is None:
            if self.verbose: self.logger("Position {} beyond physical range. Restoring previous pose".format([config.cords],[i.deg for i in config.euler_angles]))
            return self.angles
            
        # Check all joint angles are within limits
        if all([self.joint_limits[i](val.rad) for i,val in enumerate(joint_angles)]):
            # TO-DO: implement check to make sure that angles don't flip 180 degrees 
            self.update_current_pose(config)
            return joint_angles
        else:
            self.logger("A joint is hitting its limit. Restoring previous pose".format([config.cords],[i.deg for i in config.euler_angles]))
            return self.angles
            
    def OLD_not_safe_IK(self, config=None):
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
        