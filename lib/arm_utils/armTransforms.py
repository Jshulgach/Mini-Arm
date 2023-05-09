#from dataclasses import dataclass
#from typing import List

import ulab.numpy as np

__author__ = "Alberto Abarzua"


#@dataclass
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


#@dataclass
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


def tmatrix(R, D):
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
        return [Angle(float(A1), "rad"), Angle(float(B1), "rad"), Angle(float(C1), "rad")]
    else:
        C = 0
        if (R[2, 0] == -1):
            B = np.pi / 2.0
            A = C + np.arctan2(R[0, 1], R[0, 2])
        else:
            B = -np.pi / 2.0
            A = -C + np.arctan2(-R[0, 1], -R[0, 2])
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


class OutOfBoundsError(Exception):
    """OutOfBoundsEror, used to raise an exception when inverse kinematics does not 
    find a solution within the restraints.

    """

    def __init__(self, config, message="Out of bounds."):
        """Creates a new exception, with a set of angles as input and a message.

        Args:
            message (str, optional): eror message.. Defaults to "Out of bounds.".
        """
        super().__init__(message)
        self.config = config
        self.message = message

    def __str__(self):
        return self.message

def rotationFromBaseToGripper(alpha, beta, gamma):
    # Rotation of urdf_gripper with respect to the base frame interms of alpha = yaw, beta = pitch, gamma = roll
    R0u = np.array([
        [1.000*np.cos(alpha)*np.cos(beta), -1.000*np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha), 1.0*np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)],
        [1.000*np.sin(alpha)*np.cos(beta),  np.sin(alpha)*np.sin(beta)*np.sin(gamma) + 1.000*np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma) - 1.0*np.sin(gamma)*np.cos(alpha)],
        [          -1.000*np.sin(beta),                                     1.000*np.sin(gamma)*np.cos(beta),                                    1.0*np.cos(beta)*np.cos(gamma)]])
    return R0u

def get_wrist_center(gripper_point, R0g, dg = 0.303):
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


