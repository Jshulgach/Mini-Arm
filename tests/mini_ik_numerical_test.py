# Inverse kinematic test using numerical methods, no symbolic math

import numpy as np
from numpy import array, sin, cos, isclose, arctan2, sqrt, pi, dot

eps = 0.00000001


def xmatrix(angle):
    r = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])
    return r


def ymatrix(angle):
    r = np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])
    return r


def zmatrix(angle):
    r = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    return r
    
def eulerAnglesToRotMatrix(A, B, C):
    return dot(dot(zmatrix(C), ymatrix(B)), xmatrix(A))
    

def get_wrist_center(gripper_point, R0g, dg = 0.03):

    xu, yu, zu = gripper_point     
    nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
    xw = xu - dg * nx
    yw = yu - dg * ny
    zw = zu - dg * nz 
    return xw, yw, zw

def get_hypotenuse(a, b):
    return sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):
    cos_gamma = (a*a + b*b - c*c) / (2*a*b)
    sin_gamma = sqrt(1 - cos_gamma * cos_gamma)
    gamma = arctan2(sin_gamma, cos_gamma)

    return gamma

def get_first_three_angles(wrist_center):

    x, y, z  = wrist_center
    
    a1 = 0.015
    a2 = 0.12
    a3 = 0.0
    d1 = 0.095
    d4 = 0.09
    l = get_hypotenuse(d4, -a3)
    phi = arctan2(d4, -a3)
  
    x_prime = get_hypotenuse(x, y)
    mx = x_prime -  a1
    my = z - d1 
    m = get_hypotenuse(mx, my)
    alpha = arctan2(my, mx)
  
    gamma = get_cosine_law_angle(l, a2, m)
    beta = get_cosine_law_angle(m, a2, l)
  
    #q1 = arctan2(y, x)
    #q2 = pi/2 - beta - alpha 
    #q3 = -(gamma - phi)
    # Round off values to zero 
    q1 = 0.000 if abs(np.arctan2(y, x)) < eps else np.arctan2(y, x)
    q2 = 0.000 if abs(np.pi/2 - beta - alpha) < eps else np.pi/2 - beta - alpha
    q3 = 0.000 if abs(-(gamma - phi)) < eps else -(gamma - phi)
 
    return q1, q2, q3 

def get_last_three_angles(R):
    
    sin_q4 = R[2, 2]
    cos_q4 =  -R[0, 2]
    if cos_q4 == 0: cos_q4 = 0.0

    sin_q5 = sqrt(R[0, 2]**2 + R[2, 2]**2) 
    cos_q5 = R[1, 2]
    
    sin_q6 = -R[1, 1]
    if sin_q6 == 0: sin_q6 = 0.0
    cos_q6 = R[1, 0] 
  
    q4 = arctan2(sin_q4, cos_q4)
    q5 = arctan2(sin_q5, cos_q5)
    q6 = arctan2(sin_q6, cos_q6)

    # because this model has some singularities, we may consider
    if( np.sin(q5) < 0 ):
        q4 = np.arctan2(-sin_q4, cos_q4)
        q6 = np.arctan2(sin_q6, -cos_q6)
    
    
    return q4, q5, q6

 # ======= Matices are declared below =========
 
def R03(q1, q2, q3):
    r03 = array([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
                 [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
                 [        cos(q2 + q3),        -sin(q2 + q3),        0]])
    return r03

def R03T(q1, q2, q3):
  # Transpose of R03 
    r03t = array([[sin(q2 + q3)*cos(q1), sin(q1)*sin(q2 + q3),  cos(q2 + q3)],
                  [cos(q1)*cos(q2 + q3), sin(q1)*cos(q2 + q3), -sin(q2 + q3)],
                  [            -sin(q1),              cos(q1),             0]])
    return r03t

def R36(q4, q5, q6):
  # Rotation of joint 6 wrt to frame of joint 3 interms of the last three angles q4, q5, q6
    r36 = array([[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
                 [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
                 [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
    return r36

def R0u(alpha, beta, gamma):
    # Rotation of urdf_gripper with respect to the base frame interms of alpha = yaw, beta = pitch, gamma = roll
    r0u = array([[1.0*cos(alpha)*cos(beta), -1.0*sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), 1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)],
                 [1.0*sin(alpha)*cos(beta),  sin(alpha)*sin(beta)*sin(gamma) + 1.0*cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha)],
                 [          -1.0*sin(beta),                                     1.0*sin(gamma)*cos(beta),                                    1.0*cos(beta)*cos(gamma)]])
    return r0u

def T0g_b(alpha, beta, gamma):
    # Total transform of gripper wrt to base frame given orientation yaw (alpha), pitch (beta), roll (beta) and position px, py, pz
    t0g_b = array([[1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma),  1.0*sin(alpha)*cos(gamma) - 1.0*sin(beta)*sin(gamma)*cos(alpha), 1.0*cos(alpha)*cos(beta), px],
                   [sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha), -1.0*sin(alpha)*sin(beta)*sin(gamma) - 1.0*cos(alpha)*cos(gamma), 1.0*sin(alpha)*cos(beta), py],
                   [                                   1.0*cos(beta)*cos(gamma),                                        -1.0*sin(gamma)*cos(beta),           -1.0*sin(beta), pz],
                   [                                                          0,                                                                0,                        0,  1]])
    return t0g_b

def T0g_a(q1, q2, q3, q4, q5, q6):
    # Total transform of gripper wrt to base frame given angles q1, q2, q3, q4, q5, q6
    t0g_a = array([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
                   [ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
                   [                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
                   [                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]])
    return t0g_a

# Rotation of urdf_gripper wrt (DH) gripper frame from rotz(pi) * roty(-pi/2) and it's transpose
Rgu_eval = array([[0, 0, 1], [0, -1.00000000000000, 0], [1.00000000000000, 0, 0]])
RguT_eval = array([[0, 0, 1], [0, -1.00000000000000, 0], [1.00000000000000, 0, 0]])

def test_IK(xyz, abc):
    print(" ============= New IK test =============")
    print("position: {}".format(xyz))
    print("euler: {}".format(abc))

    gripper_point = xyz
    roll, pitch, yaw = abc

    R0u_eval = R0u(yaw, pitch, roll)
    print("R0u")
    print(R0u_eval)
    R0g_eval = dot(R0u_eval, RguT_eval)
    print("R0g")
    print(R0g_eval)

    wrist_center = get_wrist_center(gripper_point, R0g_eval, dg = 0.03)
    print("wrist_center")
    print(wrist_center)

    j1, j2, j3 = get_first_three_angles(wrist_center)
    print("first 3 joints: {} {} {}".format(j1, j2, j3))

    R03T_eval = R03T(j1, j2, j3)
    print("R03T")
    print(R03T_eval)
    R36_eval = dot(R03T_eval, R0g_eval).squeeze()
    print("R36")
    print(R36_eval)

    j4, j5, j6 = get_last_three_angles(R36_eval)
  
    return [j1, j2, j3, j4, j5, j6]


result = test_IK([0.135, 0.0, 0.155], [0.0, 0.0, 0.0])
print("result: {}".format(result))
sol = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
is_same = all(isclose(array(result, dtype=float), sol, rtol=eps))
print(" pass: {}".format(is_same))


# All tests pass....