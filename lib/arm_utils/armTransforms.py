
import ulab.numpy as np

# Matrix mathematics dont seem to be as accurate as the symbolic math or numpy operations
# on the PC. Small numbers are found in matrices instead of 0, which could be causing jumping
# values from joint states. Setting an epsilon number will guarant matrix elements should be zero
eps = 0.0001

class Angle:
    """Angle class, used to store angle values in different units.
    Radians, and degrees.
    """
    def __init__(self, value=0.0, unit="rad") -> None:
        self.value = value
        self.unit = unit

    @property
    def rad(self):
        if self.unit == "rad":
            return self.value
        return (np.pi / 180) * self.value

    @property
    def deg(self):
        if self.unit == "deg":
            return self.value
        return (180 / np.pi) * self.value

    def __eq__(self, value):
        if type(value) == Angle:
            if self.unit == "rad":
                return self.value == value.rad
            return self.value == value.deg
        return False

    def add(self, other):
        if type(other) == Angle:
            if self.unit == "rad":
                self.value += other.rad
            if self.unit == "deg":
                self.value += other.deg

    def sub(self, other):
        if type(other) == Angle:
            if self.unit == "rad":
                value = self.value - other.rad
                return Angle(value, "rad")
            if self.unit == "deg":
                value = self.value - other.deg
                return Angle(value, "deg")
        return other

class Config:
    """Pose dataclass, used to store all the information required to determine the position and configuration of 
    the robot arm. (coordinates, euler angles of the tcp and the state of the tool (for now None))
    """
    def __init__(self, cords=[float], euler_angles=[Angle], tool=100):
        self.cords = cords
        self.euler_angles = euler_angles
        self.tool = tool

    def __str__(self):
        return "  cords: [x: {:.5f}, y: {:.5f}, z: {:.5f}]\n  angles: [Roll: {:.5f}, Pitch: {:.5f}, Yaw:{:.5f}]\n  tool: {}".format(
            *(self.cords[0:3] + [x.deg for x in self.euler_angles] + [self.tool]))


def rad2degree(angle):
    """Converts angles in radians to degrees

    Args:
        angle (float): angle we want to convert for radians to degree

    Returns:
        float: angle in degrees
    """
    return (180 / np.pi) * angle


def degree2rad(angle):
    """Converts angles in degrees to radian

    Args:
        angle (float): angle in degrees

    Returns:
        float: angle in radians
    """
    return (np.pi / 180) * angle


def xmatrix(angle):
    """Creates a rotation matrix using angle about the X axis.

    Args:
        angle (Angle): angle that must be in radians

    Returns:
        np.array: 3x3 transformation matrix using angle
    """
    angle = angle.rad
    r = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])
    return r


def ymatrix(angle):
    """Creates a rotation matrix using angle about the Y axis.

    Args:
        angle (Angle): angle that must be in radians

    Returns:
        np.array: 3x3 transformation matrix using angle
    """
    angle = angle.rad
    r = np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])
    return r


def zmatrix(angle):
    """Creates a rotation matrix using angle about the Z axis.

    Args:
        angle (Angle): angle that must be in radians

    Returns:
       np.array: 3x3 transformation matrix using angle
    """
    angle = angle.rad
    r = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    return r

def enforcePositiveServoRange(angle):
    """ Checks the incoming rotation Angle value and checks its range to be between 0-to-pi. If out of the
    range it remaps it to the closest angle reflected from the 0-pi axis
    
    Parameters:
    angle    : (Angle) Angle value
    """
    if angle.rad > np.pi or angle.rad < -np.pi:
        angle = Angle(2*np.pi - abs(angle.rad), "rad")
    if angle.rad < 0: # and angle.rad > -np.pi:
        angle = Angle(abs(angle.rad), "rad")
    
    return angle

def TransformationMatrix(R, D):
    """Creates a homogeneus transformation matrix using a translation and a rotation matrix.

    Args:
        R (np.array): 3X3 Rotation matrix used in the transformation    
        D (np.array): Array of size 3 that contains the translation in x,y,z

    Returns:
        np.array: Homogeneus transformation matrix.
    """
    #T = np.block([
    #    [R, D],
    #    [0, 0, 0, 1]
    #])
    temp = np.concatenate((R, D), axis=1)
    T = np.concatenate((temp, np.array([[0, 0, 0, 1]])), axis=0)
    return T

def AddisonTransformationMatrixDH(theta, alpha, a, d):
    """ Creates a homogenous transformation matrix of one joint frame with respect to the 
        previous joint frame using the Denavit-Hartenberg convention. Ues the exmple 
        homogenous matrix structure from Angela Sodemann, referenced by Automatic Addison
        
        Building DH tables:
            https://automaticaddison.com/how-to-find-denavit-hartenberg-parameter-tables/
            https://automaticaddison.com/coding-denavit-hartenberg-tables-using-python/    
            
    Parameters:
    -----------
    theta  : Angle from joint[i-1] to joint[i] around z[i-1]
    alpha  : Angle from joint[i-1] to joint[i] around x[i]
    a      : Distance between previous frame and current frame along x[i] direction
    d      : Distance from x[i-1] to x[i] along the z[i-1] direction
    
    Return:
    --------
    T      : Numpy array with the homogenous transformation matrix
    """
    
    t11 = np.cos(theta)
    t12 = -np.sin(theta) * np.cos(alpha)
    t13 = np.sin(theta) * np.sin(alpha)
    t14 = a * np.cos(theta)
    t21 = np.sin(theta)
    t22 = np.cos(theta) * np.cos(alpha)
    t23 = -np.cos(theta) * np.sin(alpha) 
    t24 = a * np.sin(theta)
    t31 = 0.000
    t32 = np.sin(alpha)
    t33 = np.cos(alpha)
    t34 = d
    T = np.array([[  t11,   t12,   t13,    t14],
                  [  t21,   t22,   t23,    t24],
                  [  t31,   t32,   t33,    t34],
                  [0.000, 0.000, 0.000, 1.000]])
    
    # Enforce epsilon, operation implemented for 1D boolean array only
    for i, row in enumerate(T):
        T[i][abs(T[i]) < eps] = 0.00

    return T
    
def TransformationMatrixDH(theta, alpha, a, d):
    """ Creates a homogenous transformation matrix of one joint frame with respect to the 
        previous joint frame using the Denavit-Hartenberg convention. Uses the structure from 
        Mithi's arm-ik repository        
        
    Parameters:
    -----------
    theta  : Angle from joint[i-1] to joint[i] around z[i-1]
    alpha  : Angle from joint[i-1] to joint[i] around x[i]
    a      : Distance between previous frame and current frame along x[i] direction
    d      : Distance from x[i-1] to x[i] along the z[i-1] direction
    
    Return:
    --------
    T      : Numpy array with the homogenous transformation matrix
    """
    
    r11, r12 = np.cos(theta), -np.sin(theta)
    r23, r33 = -np.sin(alpha), np.cos(alpha)
    r21 = np.sin(theta) * np.cos(alpha)
    r22 = np.cos(theta) * np.cos(alpha)
    r31 = np.sin(theta) * np.sin(alpha)
    r32 = np.cos(theta) * np.sin(alpha)
    y = -d * np.sin(alpha)
    z = d * np.cos(alpha)
    
    T = np.array([
                 [r11,     r12, 0.000,     a],
                 [r21,     r22,   r23,     y],
                 [r31,     r32,   r33,     z],
                 [0.000, 0.000, 0.000, 1.000]
                 ])
    
    # Enforce epsilon, operation implemented for 1D boolean array only
    for i, row in enumerate(T):
        T[i][abs(T[i]) < eps] = 0.00

    return T


def rotationMatrixToEulerAngles(R):
    """Used to get the euler angles from a rotation matrix. (roll, pitch and yaw)

    Args:
        R (np.array): Rotation matrix

    Returns:
        (list[Angle]): The three euler angles obtained from the rotation matrix
    """
    if (R[2, 0] != 1 and R[2, 0] != -1):
        B1 = -np.asin(R[2, 0])
        B2 = np.pi + B1
        A1 = np.arctan2(R[2, 1] / np.cos(B1), R[2, 2] / np.cos(B1))
        A2 = np.arctan2(R[2, 1] / np.cos(B2), R[2, 2] / np.cos(B2))
        C1 = np.arctan2(R[1, 0] / np.cos(B1), R[0, 0] / np.cos(B1))
        C2 = np.arctan2(R[1, 0] / np.cos(B2), R[0, 0] / np.cos(B2))
        
        # Enforce epsilon, operation implemented for 1D boolean array only
        if abs(A1) < eps: A1 = 0.000
        if abs(B1) < eps: B1 = 0.000
        if abs(C1) < eps: C1 = 0.000

        return [Angle(float(A1), "rad"), Angle(float(B1), "rad"), Angle(float(C1), "rad")]
    else:
        C = 0
        if (R[2, 0] == -1):
            B = np.pi / 2.0
            A = C + np.arctan2(R[0, 1], R[0, 2])
        else:
            B = -np.pi / 2.0
            A = -C + np.arctan2(-R[0, 1], -R[0, 2])

        # Enforce epsilon, operation implemented for 1D boolean array only
        if abs(A) < eps: A = 0.000
        if abs(B) < eps: B = 0.000
        if abs(C) < eps: C = 0.000

        return [Angle(float(A), "rad"), Angle(float(B), "rad"), Angle(float(C), "rad")]


def eulerAnglesToRotMatrix(A, B, C):
    """Gets a rotation matrix from three euler angles

    Args:
        A (Angle): x-euler Angle    
        B (Angle): y-euler Angle    
        C (Angle): z-euler Angle

    Returns:
        np.array: 3x3 rotation matrix
    """
    #return zmatrix(C) @ ymatrix(B) @ xmatrix(A)
    return np.dot(np.dot(zmatrix(C), ymatrix(B)), xmatrix(A))


def angle_list(angles, unit):
    """Creates list of Angles from array of float values for an angle.


    Args:
        angles (list[float]): list of angles in float
        unit (str): unit (radians or degrees)

    Returns:
        list[Angle]: list of angles in the Angle data type
    """
    return [Angle(angle, unit) for angle in angles]


def nearest_to_prev(angles, prev):
    """Uses nearest_to_prev_aux on a list of Angles and applies it to all of the angles
    insise de list

    Args:
        angles (list[Angle]): list of angles
        prev (list[Angle]): List of previous angles.

    Returns:
        list[Angle]: list of modified angles.
    """

    rad_angles = [nearest_to_prev_aux(angle.rad, pre.rad) for angle, pre in zip(angles, prev)]
    result = [Angle(angle, "rad") for angle in rad_angles]

    return result


def nearest_to_prev_aux(angle, prev):
    """Returns the nearest angle to prev that follows this formula
    new_angle = 2*n*np.pi + angle, where n is an integer.

    Args:
        angle (float): angle in radians
        prev (float): List of previous angles.
    Returns:
        (float): angle that is nearest to zero.
    """
    #print("armTransforms angle: {}".format(angle))
    #print("armTransforms prev: {}".format(prev))
    if (abs(angle - prev) <= np.pi):
        return angle
    if (abs(angle + np.pi * 2 - prev) < abs(angle - prev)):
        return nearest_to_prev_aux(2 * np.pi + angle, prev)
    if (abs(angle - np.pi * 2 - prev) < abs(angle - prev)):
        return nearest_to_prev_aux(angle - 2 * np.pi, prev)


def generate_curve_ptp(p1, p2, n=None):
    """Creates a parametric straight curve from point p1 to point p2.

    Args:
        p1 (Config): starting point
        p2 (Config): ending point
        n (int) : Number of points in the curve (definition)

    Returns:
        np.array: Array with all the positions to achieve the curve.
        (Dimensions: (self.numJoints+1),n)
    """
    cords_ini, euler_ini = p1.cords, p1.euler_angles
    cords_fin, euler_fin = p2.cords, p2.euler_angles
    tool_ini, tool_fin = p1.tool, p2.tool
    cords_ini = np.array(cords_ini)
    cords_fin = np.array(cords_fin)
    euler_ini = np.array([angle.rad for angle in euler_ini])
    euler_fin = np.array([angle.rad for angle in euler_fin])
    if (n == None):
        dist1 = np.linalg.norm(cords_ini - cords_fin)
        dist2 = np.linalg.norm(euler_ini - euler_fin) * 60
        dist = max(dist1, dist2, 1 / 5)
        n = int(dist * 0.2)
    t = 0
    if (n == 0):  # This means only the tool was changed
        n = abs(tool_ini - tool_fin)
    step = 1 / n
    result = np.zeros((n + 1, 7))
    for i in range(n + 1):
        cur_cords = cords_ini * (1 - t) + cords_fin * (t)
        cur_tool = tool_ini * (1 - t) + tool_fin * (t)
        cur_euler = euler_ini * (1 - t) + euler_fin * (t)
        cur_post = np.append(cur_cords, cur_euler, axis=0)
        cur_post = np.append(cur_post, np.array([cur_tool]), axis=0)
        cur_post = np.transpose(cur_post)
        t += step
        result[i] += cur_post
    return result


def gen_curve_points(points):
    """Generates curves ptp from a list of points

    Args:
        points (list[Config]): list of points

    Returns:
        np.array: array with all the positions the tcp should follow
    """
    sleep_idxs = []
    result = None
    for i in range(len(points) - 1):
        if (i == 0):
            result = generate_curve_ptp(points[i], points[i + 1])
            continue
        sleep_idxs.append(len(result))
        result = np.append(result, generate_curve_ptp(
            points[i], points[i + 1]), axis=0)
    return result, sleep_idxs


def rotationFromBaseToGripper(alpha, beta, gamma):
    # Rotation of urdf_gripper with respect to the base frame interms of alpha = yaw, beta = pitch, gamma = roll
    R0u = np.array([
        [1.000*np.cos(alpha)*np.cos(beta), -1.000*np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha), 1.0*np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)],
        [1.000*np.sin(alpha)*np.cos(beta),  np.sin(alpha)*np.sin(beta)*np.sin(gamma) + 1.000*np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma) - 1.0*np.sin(gamma)*np.cos(alpha)],
        [             -1.000*np.sin(beta),                                              1.000*np.sin(gamma)*np.cos(beta),                                             1.0*np.cos(beta)*np.cos(gamma)]])
    return R0u


def get_wrist_center(gripper_point, R0g, dg = 0.100):
    # get the coordinates of the wrist center wrt to the base frame (xw, yw, zw)
    # given the following info:
    # the coordinates of the gripper (end effector) (x, y, z)
    # the rotation of the gripper in gripper frame wrt to the base frame (R0u)
    # the distance between gripper and wrist center dg which is along common z axis
    # check WRITEUP.pdf for more info
    xu, yu, zu = gripper_point 
    
    nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
    xw = xu - dg * nx
    yw = yu - dg * ny
    zw = zu - dg * nz 

    return xw, yw, zw


def create_circular_trajectory(center, radius=10, steps=101):
    theta = np.linspace(0, 2*np.pi, steps)
    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
    return center + radius*temp
    
def get_hypotenuse(a, b):
  return np.sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):    
    cos_gamma = (a*a + b*b - c*c) / (2*a*b)
    sin_gamma = np.sqrt(1 - cos_gamma * cos_gamma)
    gamma = np.arctan2(sin_gamma, cos_gamma)
    return gamma
  
def dot_product_chain_of_matrices(matrices):
    """Computes the dot product of a chain of matrices.

    Args:
      matrices: A list of matrices.

    Returns:
      The dot product of the chain of matrices.
    """
    product = np.eye(matrices[0].shape[0])
    for matrix in matrices:
        product = np.dot(product, matrix)
        
    # Enforce epsilon, operation implemented for 1D boolean array only
    for i, row in enumerate(product):
        product[i][abs(product[i]) < eps] = 0.00
        
    return product
    
def eulToRotMat(theta1, theta2, theta3, order='xyz'):
    """
    input
        theta1, theta2, theta3 = rotation angles in rotation order (degrees)
        oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'
    output
        3x3 rotation matrix (numpy array)
    """
    c1 = 0.0 if abs(np.cos(theta1.rad * np.pi / 180)) < eps else np.cos(theta1.rad * np.pi / 180)
    s1 = 0.0 if abs(np.sin(theta1.rad * np.pi / 180)) < eps else np.sin(theta1.rad * np.pi / 180)
    c2 = 0.0 if abs(np.cos(theta2.rad * np.pi / 180)) < eps else np.cos(theta2.rad * np.pi / 180)
    s2 = 0.0 if abs(np.sin(theta2.rad * np.pi / 180)) < eps else np.sin(theta2.rad * np.pi / 180)
    c3 = 0.0 if abs(np.cos(theta3.rad * np.pi / 180)) < eps else np.cos(theta3.rad * np.pi / 180)
    s3 = 0.0 if abs(np.sin(theta3.rad * np.pi / 180)) < eps else np.sin(theta3.rad * np.pi / 180)

    if order == 'xzx':
        matrix=np.array([[c2, -c3*s2, s2*s3],
                         [c1*s2, c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3],
                         [s1*s2, c1*s3+c2*c3*s1, c1*c3-c2*s1*s3]])
    elif order=='xyx':
        matrix=np.array([[c2, s2*s3, c3*s2],
                         [s1*s2, c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1],
                         [-c1*s2, c3*s1+c1*c2*s3, c1*c2*c3-s1*s3]])
    elif order=='yxy':
        matrix=np.array([[c1*c3-c2*s1*s3, s1*s2, c1*s3+c2*c3*s1],
                         [s2*s3, c2, -c3*s2],
                         [-c3*s1-c1*c2*s3, c1*s2, c1*c2*c3-s1*s3]])
    elif order=='yzy':
        matrix=np.array([[c1*c2*c3-s1*s3, -c1*s2, c3*s1+c1*c2*s3],
                         [c3*s2, c2, s2*s3],
                         [-c1*s3-c2*c3*s1, s1*s2, c1*c3-c2*s1*s3]])
    elif order=='zyz':
        matrix=np.array([[c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2],
                         [c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2],
                         [-c3*s2, s2*s3, c2]])
    elif order=='zxz':
        matrix=np.array([[c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1, s1*s2],
                         [c3*s1+c1*c2*s3, c1*c2*c3-s1*s3, -c1*s2],
                         [s2*s3, c3*s2, c2]])
    elif order=='xyz':
        matrix=np.array([[c2*c3, -c2*s3, s2],
                         [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                         [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
    elif order=='xzy':
        matrix=np.array([[c2*c3, -s2, c2*s3],
                         [s1*s3+c1*c3*s2, c1*c2, c1*s2*s3-c3*s1],
                         [c3*s1*s2-c1*s3, c2*s1, c1*c3+s1*s2*s3]])
    elif order=='yxz':
        matrix=np.array([[c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1],
                         [c2*s3, c2*c3, -s2],
                         [c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2]])
    elif order=='yzx':
        matrix=np.array([[c1*c2, s1*s3-c1*c3*s2, c3*s1+c1*s2*s3],
                         [s2, c2*c3, -c2*s3],
                         [-c2*s1, c1*s3+c3*s1*s2, c1*c3-s1*s2*s3]])
    elif order=='zyx':
        matrix=np.array([[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2],
                         [c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3],
                         [-s2, c2*s3, c2*c3]])
    elif order=='zxy':
        matrix=np.array([[c1*c3-s1*s2*s3, -c2*s1, c1*s3+c3*s1*s2],
                         [c3*s1+c1*s2*s3, c1*c2, s1*s3-c1*c3*s2],
                         [-c2*s3, s2, c2*c3]])

    return matrix
    
def rotMatToEul(matrix, order='xyz'):
    """
    input
        matrix = 3x3 rotation matrix (numpy array)
        oreder(str) = rotation order of x, y, z : e.g, rotation XZY -- 'xzy'
    output
        theta1, theta2, theta3 = rotation angles in rotation order
    """
    r11, r12, r13 = matrix[0,0], matrix[0, 1], matrix[0, 2] #matrix[0]
    r21, r22, r23 = matrix[1,0], matrix[1, 1], matrix[1, 2] #matrix[1]
    r31, r32, r33 = matrix[2,0], matrix[2, 1], matrix[2, 2] #matrix[2]

    if order == 'xzx':
        theta1 = np.atan(r31 / r21)
        theta2 = np.atan(r21 / (r11 * np.cos(theta1)))
        theta3 = np.atan(-r13 / r12)

    elif order == 'xyx':
        theta1 = np.arctan2(-r21 / r31)
        theta2 = np.arctan2(-r31 / (r11 *np.cos(theta1)))
        theta3 = np.arctan2(r12 / r13)

    elif order == 'yxy':
        theta1 = np.arctan2(r12 / r32)
        theta2 = np.arctan2(r32 / (r22 *np.cos(theta1)))
        theta3 = np.arctan2(-r21 / r23)

    elif order == 'yzy':
        theta1 = np.arctan2(-r32 / r12)
        theta2 = np.arctan2(-r12 / (r22 *np.cos(theta1)))
        theta3 = np.arctan2(r23 / r21)

    elif order == 'zyz':
        theta1 = np.arctan2(r23 / r13)
        theta2 = np.arctan2(r13 / (r33 *np.cos(theta1)))
        theta3 = np.arctan2(-r32 / r31)

    elif order == 'zxz':
        theta1 = np.arctan2(-r13 / r23)
        theta2 = np.arctan2(-r23 / (r33 *np.cos(theta1)))
        theta3 = np.arctan2(r31 / r32)

    elif order == 'xzy':
        theta1 = np.atan(r32 / r22)
        theta2 = np.atan(-r12 * np.cos(theta1) / r22)
        theta3 = np.atan(r13 / r11)

    elif order == 'xyz':
        if (r31 != 1 and r31 != -1):
            theta1 = np.atan(-r23 / r33)
            theta2 = np.atan(r13 * np.cos(theta1) / r33)
            theta3 = np.atan(-r12 / r11)
        else:
            theta3 = 0.0
            if (r31 == -1):
                theta2 = np.pi / 2.0
                theta1 = theta3 + np.arctan2(r12, r13)
            else:
                theta2 = -np.pi / 2.0
                theta1 = -theta3 + np.arctan2(-r12, -r13)

    elif order == 'yxz':
        theta1 = np.arctan2(r13 / r33)
        theta2 = np.arctan2(-r23 * np.cos(theta1) / r33)
        theta3 = np.arctan2(r21 / r22)

    elif order == 'yzx':
        theta1 = np.arctan2(-r31 / r11)
        theta2 = np.arctan2(r21 * np.cos(theta1) / r11)
        theta3 = np.arctan2(-r23 / r22)

    elif order == 'zyx':
        theta1 = np.arctan2(r21 / r11)
        theta2 = np.arctan2(-r31 * np.cos(theta1) / r11)
        theta3 = np.arctan2(r32 / r33)

    elif order == 'zxy':
        theta1 = np.arctan2(-r12 / r22)
        theta2 = np.arctan2(r32 * np.cos(theta1) / r22)
        theta3 = np.arctan2(-r31 / r33)

    theta1 = theta1 * 180 / np.pi
    theta2 = theta2 * 180 / np.pi
    theta3 = theta3 * 180 / np.pi
    
    if abs(theta1) < eps: theta1 = 0.000
    if abs(theta2) < eps: theta2 = 0.000
    if abs(theta3) < eps: theta3 = 0.000


    return [Angle(float(theta1), "rad"), Angle(float(theta2), "rad"), Angle(float(theta3), "rad")]
    