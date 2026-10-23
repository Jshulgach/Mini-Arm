import time
import math
from arm_utils.armTransforms import *

__author__ = "Jonathan Shulgach"
__version__ = "1.4.5"
__HOME__ = [0, 0, 0, 0, 0, 0] # Home state for robot joints, after taking offset into consideration
#__HOME__   = [0, 32,  0, 90, 15, 90]  # Home position from AJ Unity GUI 12/3/23
__IKHOME__ = [0,  0,  0,  0,  0,  0]  # IK transformations

# PCA9685 mapping recovered from the connected Pico 2 W on 2026-09-06.
# Preserved existing wiring settings; physical calibration remains unverified.
BASE_SERVO           = 15
SHOULDER_SERVO       = 14
ELBOW_SERVO          = 13
WRIST_ROT_SERVO      = 12
WRIST_BEND_SERVO     = 2
GRIPPER_LINK_SERVO   = 1
GRIPPER_SERVO        = 0

# Set values for open/close states of gripper if keyword is passed
GRIPPER = {'open': 120, 'close': 0} # Open used to be 100

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
                       verbose=False,
                       ):
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
        self.flip_direction = [True, True, False, False, False, False, False] # 12/27/24
        #self.flip_direction = [False, True, False, False, False, False, False] # 12/4/23
        self.orient_flip  = [False, False, False, False, True, False] # Preserve installed elbow direction
        self.joint_offsets = [90, 90, 45, 90, 110, 90, 90] # Necessary for the IK math to fix the transformations
        # For Unity GUI 12/14/23
        #self.flip_direction = [False, True, False, False, False, False, False] # 12/4/23
        #self.orient_flip  = [False, False, False, False, False, False]
        #self.joint_offsets = [0, 0, 0, 0, 0, 0, 90]

        self.joint_limits = [lambda x: x > self.default_min and x < self.default_max for x in range(0,6)]
        self.joint_speed_limit = 120 # rpm,  Max speed is 120 revolutions per minute

        # Joints angles
        self.angles = [Angle(0.00, "rad") for i in range(6)] # Used for tracking the joint state
        self.config = Config([], [], 0) # Used for tracking the end effector pose
        self.reference_pos = None

        # Create references to the servo hardware
        self.servos = None
        if not self.simulate_hardware:
            # necessary to create the custom busio.I2C object instead of the ServoKit default due to some dormant library issue
            import busio
            import board
            from adafruit_servokit import ServoKit
            i2c = busio.I2C(board.GP1, board.GP0)
            self.servos = ServoKit(channels=16, i2c=i2c)
            # PCA9685 can retain pulses across a Pico soft restart.
            # Clear every configured output before accepting commands.
            for joint in range(1, 8):
                self.servos.servo[ServoIndex(joint)].angle = None

    @property
    def constraints(self):
        return [self.j1_range, self.j2_range, self.j3_range, self.j4_range, self.j5_range, self.j6_range]

    def calibration_outputs(self):
        """Seven PWM command states, never measured shaft positions."""
        if self.simulate_hardware:
            return [None] * 7
        return [self.servos.servo[ServoIndex(i)].angle for i in range(1, 8)]

    def calibration_base_write(self, angle):
        """Raw base-only calibration output; does not invent a model pose."""
        if BASE_SERVO != 15:
            raise ValueError('Base calibration requires the recovered channel 15 mapping')
        if angle is not None and (isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or not 0 <= angle <= 180):
            raise ValueError('Invalid base servo target')
        if not self.simulate_hardware:
            self.update_servo(BASE_SERVO, angle)

    def calibration_shoulder_write(self, angle):
        """Raw shoulder-only desk test; not a model-space command."""
        if SHOULDER_SERVO != 14:
            raise ValueError('Shoulder calibration requires channel 14')
        if angle is not None and (isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or not 0 <= angle <= 180):
            raise ValueError('Invalid shoulder servo target')
        if not self.simulate_hardware:
            self.update_servo(SHOULDER_SERVO, angle)

    def calibration_joint_write(self, joint, angle):
        """One raw servo output, with no model conversion or automatic motion."""
        channels = {'base': (BASE_SERVO, 15), 'shoulder': (SHOULDER_SERVO, 14),
                    'elbow': (ELBOW_SERVO, 13), 'wrist-rotation': (WRIST_ROT_SERVO, 12),
                    'wrist-bend': (WRIST_BEND_SERVO, 2), 'gripper-link': (GRIPPER_LINK_SERVO, 1),
                    'gripper': (GRIPPER_SERVO, 0)}
        if joint not in channels or channels[joint][0] != channels[joint][1]:
            raise ValueError('Unsupported joint or changed servo mapping')
        if angle is not None and (isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or not 0 <= angle <= 180):
            raise ValueError('Invalid servo target')
        if not self.simulate_hardware:
            self.update_servo(channels[joint][0], angle)

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
        self.logger("\nCurrent pose:\n {}".format(self.config))
        self.logger("Joint states:    {}".format(self.get_joint_state()))
        self.logger("Servo readings:  {}".format([self.get_servo_position(i) for i in range(0,6)]))
        self.logger("Gripper positon: {}".format(self.get_gripper()))
        self.logger("Joint min limit: {}".format(self.default_min))
        self.logger("Joint max limit: {}".format(self.default_max))
        self.logger("Speed control:   {}".format(self.speed_control))

    def get_joints(self):
        #self.logger("{}".format([angle.deg for i, angle in enumerate(self.angles)]))
        return [angle.deg for i, angle in enumerate(self.angles)]

    def get_joint_state(self):
        angles = [angle.deg for i, angle in enumerate(self.angles)]
        return [a for i,a in enumerate(angles)]

    def get_servo_position(self, i):
        """Return the last servo command, not independently measured position."""
        return self.servos.servo[ServoIndex(i+1)].angle if not self.simulate_hardware else self.angles[i].deg

    def _servo_target(self, i, val):
        """Convert a model angle (degrees) to a validated servo command.

        This checks the electrical 0-180 degree range, not collision clearance
        or a calibrated URDF-to-servo mapping.
        """
        if isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < 6:
            raise ValueError('Arm joint index must be 0-5; use set_gripper for the hand')
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
            raise ValueError('Joint angle must be a finite number in degrees')
        servo_val = self.joint_offsets[i] - val if self.orient_flip[i] else val + self.joint_offsets[i]
        if self.flip_direction[i]:
            servo_val = 180 - servo_val
        if not 0 <= servo_val <= 180:
            raise ValueError('Joint {} maps outside the servo range: {}'.format(i, servo_val))
        return servo_val

    def validate_joint_targets(self, angles):
        """Validate the entire six-joint request before any servo write."""
        if len(angles) != 6:
            raise ValueError('Expected exactly six arm joint angles')
        return [self._servo_target(i, val) for i, val in enumerate(angles)]

    def set_joint(self, i, val):
        """Set one zero-indexed arm joint; stored angles are commands, not feedback."""
        servo_val = self._servo_target(i, val)
        if not self.simulate_hardware:
            self.update_servo(ServoIndex(i + 1), servo_val)
        angles = self.get_joints()
        angles[i] = val
        self.angles = angle_list(angles, 'deg')
        self.forward_kinematics(self.angles)

    def home(self):
        """ Function that sets the joint states to the home position specified by the global variable __HOME__ """
        self.logger("Setting robot to home position")
        self.set_joints_speed_control(__HOME__) if self.speed_control else self.set_joints(__HOME__)

    def ikhome(self):
        """ Function that sets the joint states to the home position specified by the global variable __IKHOME__ """
        self.set_joints_speed_control(__IKHOME__) if self.speed_control else self.set_joints(__IKHOME__)

    def set_joints(self, new_angles):
        """Validate all targets before writing; a hardware I/O failure can still be partial."""
        self.validate_joint_targets(new_angles)
        if self.speed_control:
            self.set_joints_speed_control(new_angles)
        else:
            for i, val in enumerate(new_angles):
                self.set_joint(i, val)

    def set_joints_speed_control(self, new_angles, desired_speed=120):
        """Linear interpolation in signed model angles, with speed in RPM.

        This synchronous helper is not an interruptible trajectory controller.
        """
        self.validate_joint_targets(new_angles)
        if not math.isfinite(desired_speed) or desired_speed <= 0:
            raise ValueError('Speed must be positive and finite')
        start = self.get_joints()
        steps = 15
        speed = min(desired_speed, self.joint_speed_limit)
        duration = max(abs(target - initial) for target, initial in zip(new_angles, start)) / (speed * 6.0)
        for step in range(1, steps + 1):
            values = [initial + (target - initial) * step / steps for initial, target in zip(start, new_angles)]
            self.validate_joint_targets(values)
            for i, val in enumerate(values):
                self.set_joint(i, val)
            time.sleep(duration / steps)

    def update_servo(self, key, servo_val):
        """ Update the phsyical servo configured from the ServoKit, given specific index and angle value

        Parameters:
        -----------
        key        : (int, str) Key for the servo indices
        servo_val  : (int) New angle position
        """
        # Do not silently advance commanded state when a driver rejects a write.
        self.servos.servo[key].angle = servo_val

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

    def validate_gripper_target(self, val):
        if isinstance(val, str):
            val = GRIPPER.get(val.lower(), val)
            try:
                val = float(val)
            except ValueError:
                raise ValueError('Unknown gripper value')
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val) or not 0 <= val <= 180:
            raise ValueError('Gripper angle must be finite and between 0 and 180')
        return val

    def set_gripper(self, val):
        """Set a raw servo angle or named gripper state; no measured feedback."""
        target = self.validate_gripper_target(val)
        if not self.simulate_hardware:
            self.update_servo(ServoIndex('gripper'), target)
        self.config.tool = target

    def get_gripper(self):
        return self.config.tool

    def set_pose(self, target_pos):
        """ Set the end-effector to a new positionby giving [x,y,z] or [x,y,z,roll,pitch,yaw] command inputs. The 
        corresponding joint angles are computed from the inverse kinematics and the robot angles are updated accordingly

        Parameters:
        -----------
        target_pos : (list) numeric list of [x,y,z]  or [x, y, z, roll, pitch, yaw] coordinates

        """
        if len(target_pos) not in (3, 6, 7):
            raise ValueError('Expected xyz, xyz+rpy, or xyz+rpy+gripper')
        target_pos = [float(i) for i in target_pos]
        if not all(math.isfinite(v) for v in target_pos):
            raise ValueError('Pose values must be finite')
        try:
            if len(target_pos) < 3:
                if self.verbose: self.logger("Not enough inputs for pose. Need at least xyz")
                return

            if len(target_pos) < 6:
                prev_eul = [x.rad for x in self.config.euler_angles]
                config_msg = Config(target_pos[:3], angle_list(prev_eul, "rad"))
            else:
                config_msg = Config(target_pos[0:3], angle_list(target_pos[3:6], "deg"))

            # Validate the complete solution before changing commanded state.
            candidate = self.inverse_kinematics(config_msg)
            targets = [angle.deg for angle in candidate]
            self.validate_joint_targets(targets)
            if len(target_pos) > 6:
                self.validate_gripper_target(target_pos[6])
            self.set_joints(targets)
            if len(target_pos) > 6:
                self.set_gripper(target_pos[6])

        except Exception:
            # Reject failed IK/servo commands explicitly; do not report a new pose.
            raise

    def get_pose(self):
        """ Gets the current end-effector pose """
        self.logger("\nCurrent pose: {}".format(self.config))

    def handle_delta(self, delta_pose):
        """ Adjust the end effector position to a new relative position using the delta input.
            Caution: Expecting small inputs (0.001m to 0.01m) increments)

        Parameters:
        -----------
        delta_pose : (list) numeric list of [x,y,z] or [x,y,z, roll, pitch, yaw] coordinates in meters
        """
        print(delta_pose)
        try:
            print(delta_pose)
            delta_pose = [float(i) for i in delta_pose] # Make sure values are not strings
            target_xyz = [a + b for a, b in zip(self.config.cords[:3], delta_pose[:3])]
            target_euler = [a.deg + b for a, b in zip(self.config.euler_angles[:3], delta_pose[3:6])]

            target_pose = target_xyz + target_euler
            if len(delta_pose) > 6: target_pose.append(delta_pose[6])
            return target_pose
        except:
            self.logger("Error in handling delta command", warning=True)
            xyz = self.config.cords[:3]
            rpy = [i.deg for i in self.config.euler_angles[:3]]
            raise ValueError('Invalid delta pose')

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
        self.config.euler_angles = config.euler_angles
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
            raise ValueError('No inverse-kinematics solution for requested pose')

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
            #print("Wrist center: {}".format(wrist_center))

            # Find joints 1-3
            J1, J2, J3 = self.get_first_three_angles(wrist_center)
            #print("first 3 joints: {} {} {}".format(J1.deg, J2.deg, J3.deg))
            
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
          
