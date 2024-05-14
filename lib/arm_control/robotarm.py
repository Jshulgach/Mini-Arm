
import time
import busio
import board
from adafruit_servokit import ServoKit
from arm_utils.armTransforms import *

__author__ = "Jonathan Shulgach"
__version__ = "1.4.2"
__HOME__   = [0, 32,  0, 90, 15, 90] # Home position from AJ Unity GUI 12/3/23
__IKHOME__ = [0,  0,  0,  0,  0,  0] # IK transformations 

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

    def __init__(self, name='Robot', 
                       simulate_hardware=False, 
                       dh_params=None, 
                       speed_control=False, 
                       verbose=False):
        """Robot arm constructor, it creates the robot arm with it's physical parameters and 
        initializes with all the angles equal to zero.
        """
        self.name = name
        self.simulate_hardware = simulate_hardware
        self.dh = dh_params
        self.speed_control = speed_control
        self.verbose = verbose
        
        self.counter = 0
        self.warning = True

        # joints constraints
        self.default_min = -2*np.pi
        self.default_max = 2*np.pi
        self.flip_direction = [False, True, False, False, False, False, False] # 12/4/23
        self.orient_flip  = [False, False, True, False, True, False]
        self.joint_offsets = [90, 90, 45, 90, 90, 90, 90] # Necessary for the IK math to fix the transformations


        # For Unity GUI 12/14/23
        self.flip_direction = [False, True, False, False, False, False, False] # 12/4/23
        self.orient_flip  = [False, False, False, False, False, False]
        self.joint_offsets = [0, 0, 0, 0, 0, 0, 90] 

        #self.joint_offsets = [0, 0, 0, 0, 0, 0, 0] # Necessary for the IK math to fix the transformations
        
        self.joint_limits = [lambda x: x > self.default_min and x < self.default_max for x in range(0,6)]
        self.joint_speed_limit = 120 # rpm,  Max speed is 120 revolutions per minute

        # Joints angles
        self.angles = [Angle(0.00, "rad") for i in range(6)] # Used for tracking the joint state
        self.config = Config([], [], 100) # Used for tracking the end effector pose
        self.reference_pos = None

        # Create references to the servo hardware
        self.servos = None
        if not self.simulate_hardware:
            # necessary to create the custom busio.I2C object instead of the ServoKit default due to some dormant library issue 
            i2c = busio.I2C(board.GP1, board.GP0)
            self.servos = ServoKit(channels=16, i2c=i2c) 
            
    @property
    def constraints(self):
        return [self.j1_range, self.j2_range, self.j3_range, self.j4_range, self.j5_range, self.j6_range]

    def logger(self, *argv, warning=False):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))
        self.warning = warning

    def robotinfo(self):
        """ Helper function to display the state of the robot. The outpout will look like below:
        
        Current joint state: [90, 90, 90, 90, 90, 90]
        Current pose:   cords: [x: 185.00000, y: 0.00000, z: 215.00000]
          angles: [Roll: 0.00000, Pitch: -0.00000, Yaw:0.00000]
          tool: 100
        """
        self.logger("\nCurrent joint state: {}\nCurrent pose: {}".format(self.get_joints(),self.config))
        self.logger("Joint states:    {}".format(self.get_joint_state()))
        self.logger("Servo readings:  {}".format([self.get_servo_position(i) for i in range(0,6)]))
        self.logger("Joint min limit: {}".format(self.default_min))
        self.logger("Joint max limit: {}".format(self.default_max))
        self.logger("Speed control:   {}".format(self.speed_control))

                        
    def get_joints(self):
        return [angle.deg for i, angle in enumerate(self.angles)]
        
    def get_joint_state(self):
        angles = [angle.deg for i, angle in enumerate(self.angles)]
        return [a for i,a in enumerate(angles)]
    
    def get_servo_position(self,i):
        """ Helper function that returns the true position value for the specified servo  """        
        return self.servos.servo[ServoIndex(i+1)].angle if not self.simulate_hardware else self.angles[i].deg
        
    def set_joint(self, i, val, speed=1):
        """ Function that moves the physical motor/servo to the specified angle
        
        :param idx: (int) joint index, indexing starts at zero
        :param val: (int) angle value to set the specified joint at
        """
        
        # Check that the proper motor indexing is happenning
        if i > 6:
            self.logger("Warning - Motor indexing starts at 0")
            return
        
        # Check that the angle range is within limits of the actuator
        if not self.check_within_range(i, val):
            self.logger("Warning. Servo value passed '{}' not within range. Cancelling new joint position".format(val))
            return 
        
        if not self.simulate_hardware:
            # In addition to the servo orientation on the physical robot, the angle displacements are different
            servo_val = self.joint_offsets[i] - val if self.orient_flip[i] else val + self.joint_offsets[i]
     
            # Some actuators might be mounted in a way that the rotation direction needs to be reversed
            if self.flip_direction[i]: servo_val = 180-servo_val
            
            
            # Set the new position to the actuator. ServoIndex dictionary is only necessary for custom
            # channel mapping from the servo motor shield                
            if self.verbose: self.logger("Changing joint:{} position to: {}".format(i+1, servo_val))
            self.update_servo(ServoIndex(i+1), servo_val)
        
        angles = self.get_joints()
        angles[i] = val
        self.angles = [Angle(j, "deg") for j in angles]
                    
        # Update the internal joint state
        self.forward_kinematics(angle_list(angles, "deg"))  # Update the new position of the end-effector
                
    def home(self):
        """ Function that sets the joint states to the home position specified by the global variable __HOME__ """
        self.set_joints_speed_control(__HOME__) if self.speed_control else self.set_joints(__HOME__)
    
    def ikhome(self):
        """ Function that sets the joint states to the home position specified by the global variable __IKHOME__ """
        self.set_joints_speed_control(__IKHOME__) if self.speed_control else self.set_joints(__IKHOME__)
            
    def set_joints(self, new_angles):
        """Function that updates the joint state angles for the internal model as well as physical hardware

        Parameters:
        ------------
        new_angles    : (list) a numeric list of the new joint angles for the robot  
        """
        allpass=all([self.check_within_range(i, val) for i,val in enumerate(new_angles[0:6])])
        # Check that all joints are within limits of the actuators
        if not allpass:
            self.logger("Warning. Servo values passed {} not within range. Cancelling joint command".format(new_angles[0:6]))
            return 
        
        if self.speed_control:
            self.set_joints_speed_control(new_angles)
        else:
            self.angles = angle_list(new_angles[0:6], "deg")
            if self.verbose: self.logger("Set joint angles: {}".format(new_angles))
            for i, val in enumerate(new_angles):
                self.set_joint(i, val)
            
    def set_joints_speed_control(self, new_angles, desired_speed=120):
        """Modified function that updates the new joint state angles for the internal model as well as physical hardware

        Parameters:
        ------------
        new_angles    : (list) A numeric list of the new joint angles for the robot  
        desired_speed : (num) Desired speed in RPM (rotations per minute)
        """
     
        steps = 15 # Number of steps, can adjust as needed for smoother motion
        
        # Calculate the time delay between steps based on the desired speed
        # Ensure the speed is within the maximum limit
        speed = min(desired_speed, self.joint_speed_limit)
        
        # Calculate the delay between steps in seconds
        delay = 60.0 / (speed * steps) - 0.025
        delay = 0.001 # Or overrite it since the servos have their own incurred delay with witing a new position

        # Calculate the difference between current and target positions, for all servos
        # By getting the angle difference between the current and desired positions, we can get the position change for each incremental step
        step_sizes = [round(abs(int(new_angle) - int(abs(int(self.angles[i].deg)))) / steps) for i, new_angle in enumerate(new_angles)]
        
        # Move all servos by steps
        for _ in range(steps):

            # Calculate the step size and move each servo
            for i, new_angle in enumerate(new_angles):
                current_pos = abs(int(self.angles[i].deg))

                # Determine the direction of movement based on the difference in angles
                direction = 1 if new_angle > current_pos else -1

                # Perform movement based on calculated step sizes and direction
                new_step_pos = current_pos + step_sizes[i] * direction

                # Some actuators might be mounted in a way that the rotation direction needs to be reversed
                if self.flip_direction[i]:
                    new_step_pos = 180 - new_step_pos

                # Check that the angle range is within limits of the actuator
                if self.check_within_range(i, new_step_pos):
                    if not self.simulate_hardware:
                        servo_val = new_step_pos - self.joint_offsets[i] if self.flip_direction[i] else new_step_pos + self.joint_offsets[i]
                        #self.logger("writing servo value: {}".format(servo_val))
                        self.update_servo(ServoIndex(i + 1), servo_val)

                    angles = self.get_joints()

                    # Remember to flip back the orientation if necessary
                    angles[i] = 180 - new_step_pos if self.flip_direction[i] else new_step_pos

                    # Reassign internal joint state values
                    self.angles = [Angle(j, "deg") for j in angles]

                else:
                    if self.verbose: self.logger("Warning. Servo value passed '{}' not within range. Cancelling new joint position")

            angles = self.get_joints()
            self.forward_kinematics(angle_list(angles, "deg"))  # Update the new position of the end-effector
            time.sleep(delay)
    
    def update_servo(self, key, servo_val):
        """ Update the phsyical servo configured from the ServoKit, given specific index and angle value
                
        Parameters:
        -----------
        key        : (int, str) Key for the servo indices
        servo_val  : (int) New angle position
        """
        try:
            self.servos.servo[key].angle = servo_val 
        except ValueError as e:
            print(e)
                
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
            
            
    def check_within_range(self, i, val, radian=False):
        """ Helper function that determines whether the desired angle is within the joint limit range
        for the specified joint.
        
        Parameters:
        -----------
        i      : (int) joint index, indexing starts at zero
        val    : (int) angle value to set the specified joint at
        radian : (bool) Optional parameter to check a value in radians instead of degrees, assumes degree
        
        Returns True if within joint limit range, otherwise returns False
        """
        if not radian: val = val * np.pi / 180
        return True if self.joint_limits[i](val) else False

    def set_gripper(self, val):
        """ Function that updates only the gripper servo
        
        :param val: (int) angle value to set the specified joint at
        """
        if self.verbose: self.logger("Changing gripper position to: {}".format(val))
        self.servos.servo[ServoIndex('gripper')].angle = val
    
    
    def get_gripper(self):
        return self.config.tool
    
    
    def set_pose(self, target_pos):
        """ Set the end-effector to a new positionby giving [x,y,z] or [x,y,z,roll,pitch,yaw] command inputs. The 
        corresponding joint angles are computed from the inverse kinematics and the robot angles are updated accordingly
        
        Parameters:
        -----------
        target_pos : (list) numeric list of [x,y,z]  or [x, y, z, roll, pitch, yaw] coordinates
        
        """
        target_pos = [float(i) for i in target_pos] # Make sure values are not strings
        try:
            if len(target_pos) < 3:
                if self.verbose: self.logger("Not enough inputs for pose. Need at least xyz")
                return 
            
            if len(target_pos) < 6:
                prev_eul = [x.rad for x in self.config.euler_angles]
                config_msg = Config(target_pos[:3], angle_list(prev_eul, "rad"))
            else:
                config_msg = Config(target_pos[0:3], angle_list(target_pos[3:6], "deg"))
    
            # Update gripper if command exists
            if len(target_pos) > 6: self.set_gripper(target_pos[6])
            
            # Compute IK to find joint states
            self.angles = self.inverse_kinematics(config_msg)
                
            # Update internal model
            self.update_current_pose(config_msg)
            
            # Set servo joint positions
            self.set_joints([i.deg for i in self.angles])

        except:
            if self.verbose: self.logger("Error in setting pose. Check values are of float type and IK solver has no issues")
                
    def get_pose(self): 
        """ Gets the current end-effector pose """
        self.logger("\nCurrent pose: {}".format(self.config))
    
    def handle_delta(self, delta_pos):
        """ Adjust the end effector position to a new relative position using the delta input. 
            Expecting small inputs (0.01 increments)
        
        Parameters:
        -----------
        delta_pos : (list) numeric list of [x,y,z] or [x,y,z, roll, pitch, yaw] coordinates
        """
        try:
            delta_pos = [float(i) for i in delta_pos] # Make sure values are not strings
            target_xyz = [a + b for a, b in zip(self.config.cords[:3], delta_pos[:3])]
            target_euler = [a.deg + b for a, b in zip(self.config.euler_angles[:3], delta_pos[3:6])]
        
            target_pos = target_xyz + target_euler
            if len(delta_pos) > 6: target_pos.append(delta_pos[6])
            return  target_pos
        except: 
            self.logger("Error in handling delta command", warning=True)
            xyz = self.config.cords[:3]
            rpy = [i.deg for i in self.config.euler_angles[:3]]
            return xyz.append(rpy)
        
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

        T0g = dot_product_chain_of_matrices(self.get_transformations(angles))

        # Can get position of tool frame here
        self.config.cords = list(T0g[:3, 3])
        
        # need to adjust orientation to tool frame before saving
        
        R0g = np.dot(T0g[:3,:3], np.dot( ymatrix( Angle(np.pi/2,"rad")), zmatrix( Angle(-np.pi,"rad")) ) ) # dot product!!!

        #self.config.euler_angles = rotMatToEul(R0g)
        self.config.euler_angles = rotationMatrixToEulerAngles(R0g)
        
        # Update internal joint state
        self.angles = angles

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
        if (angles == None): angles = self.angles
            
        T = []
        for i in range(len(angles)):
            T.append(TransformationMatrixDH(angles[i].rad + self.dh[i]['theta'], self.dh[i]['alpha'], self.dh[i]['a'], self.dh[i]['d']))
            #T.append(AddisonTransformationMatrixDH(angles[i].rad + self.dh[i]['theta'], self.dh[i]['alpha'], self.dh[i]['a'], self.dh[i]['d']))
        T.append(TransformationMatrixDH(0 + self.dh[6]['theta'], self.dh[6]['alpha'], self.dh[6]['a'], self.dh[6]['d']))
        
        return T
        
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

            # ========== need to fix ==============        
            # Find gipper transformation with respect to base
            Rgu = np.dot(ymatrix(Angle(-np.pi/2, "rad")).transpose(), zmatrix(Angle(-np.pi, "rad")))   
            RguT_eval = np.array([[0, 0, 1], [0, -1.00000000000000, 0], [1.00000000000000, 0, 0]])
            R0u = eulerAnglesToRotMatrix(A, B, C)
            R0g = np.dot(R0u, Rgu.transpose()) # transform of gripper orientation with respect to robot

            # Find wrist center urdf transformation 
            wrist_center = self.get_wrist_center(xyz, R0g, self.dh[-1]['d'])
            print("Wrist center: {}".format(wrist_center))

            # Find joints 1-3
            J1, J2, J3 = self.get_first_three_angles(wrist_center)
            print("first 3 joints: {} {} {}".format(J1.deg, J2.deg, J3.deg))
            
            # Find wrist transformation with respect to base given new angles
            angles = self.angles
            prev = angles
            angles[:3] = J1, J2, J3
            T = self.get_transformations(angles)     

            # Find joints 4-6
            T03 = dot_product_chain_of_matrices(T[:3])            
            R36 = np.dot(T03[:3,:3].transpose(), R0g)
            J4, J5, J6 = self.get_last_three_angles(R36)
        
            return nearest_to_prev([J1, J2, J3, J4, J5, J6], prev)
            #return [J1, J2, J3, J4, J5, J6]
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
        #self.logger("Calculated angles: {}".format([angle.rad for angle in angles]))
        self.logger("Calculated angles: {}".format([angle.deg for angle in angles]))
        self.logger("=====================================================================")
        
    def get_wrist_center(self, gripper_point, R0g, dg = 0.303):

        xu, yu, zu = gripper_point     
        nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
        xw = xu - dg * nx
        yw = yu - dg * ny
        zw = zu - dg * nz 
        return xw, yw, zw


    def get_first_three_angles(self, wrist_center):
        
        x, y, z  = wrist_center
        a1 = self.dh[1]['a'] # self.dh[0]['a']
        a2 = self.dh[2]['a'] # self.dh[1]['a']
        a3 = self.dh[3]['a'] # self.dh[2]['a']
        d1 = self.dh[0]['d']
        d4 = self.dh[3]['d']
        
        l = get_hypotenuse(d4, -a3)
        phi = np.arctan2(d4, -a3)
  
        x_prime = get_hypotenuse(x, y)
        mx = x_prime -  a1
        my = z - d1 
        m = get_hypotenuse(mx, my)
        alpha = np.arctan2(my, mx)
  
        gamma = get_cosine_law_angle(l, a2, m)
        beta = get_cosine_law_angle(m, a2, l)

        # Round off values to zero 
        q1 = 0.000 if abs(np.arctan2(y, x)) < eps else np.arctan2(y, x)
        q2 = 0.000 if abs(np.pi/2 - beta - alpha) < eps else np.pi/2 - beta - alpha
        q3 = 0.000 if abs(-(gamma - phi)) < eps else -(gamma - phi)
     
        return Angle(q1, "rad"), Angle(q2, "rad"), Angle(q3, "rad")
        
    def get_last_three_angles(self, R):
    
        for i, row in enumerate(R):
            R[i][abs(R[i]) < eps] = 0.00
            
        # Also make sure any zero values with potentially negative terms become positive.
        # Found this to be important fpuor the joint values if the arctan has a negative sign passed in
            
        sin_q4 = R[2, 2]
        cos_q4 =  -R[0, 2]
        if cos_q4 == 0: cos_q4 = 0.0
    
        sin_q5 = np.sqrt(R[0, 2]**2 + R[2, 2]**2) 
        cos_q5 = R[1, 2]
    
        sin_q6 = -R[1, 1]
        if sin_q6 == 0: sin_q6 = 0.0
        cos_q6 = R[1, 0] 
        
        q4 = np.arctan2(sin_q4, cos_q4)
        
        q5 = np.arctan2(sin_q5, cos_q5)
        q6 = np.arctan2(sin_q6, cos_q6)
        
        # because this model has some singularities, we may consider
        if( np.sin(q5) < 0 ):
            q4 = np.arctan2(-sin_q4, cos_q4)
            q6 = np.arctan2(sin_q6, -cos_q6)
    
        # 11-19-23 adding the +-pi/2 offset

        return Angle(q4, "rad"), Angle(q5, "rad"), Angle(q6, "rad")
  

    #nearest_to_prev([J1, J2, J3, J4, J5, J6], prev)
        