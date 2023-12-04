# coding: utf-8

import numpy as np
from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix
from time import sleep

# ========= helper functions ================

# rotation matrices in x, y, z axes        
def xmatrix(angle):
    r = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]])
    return r
        
def ymatrix(angle):
    r = np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]])
    return r

def zmatrix(angle):
    r = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]])
    return r

def rotationMatrixToEulerAngles(R):
    #Reference:   https://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
    if (R[2, 0] != 1 and R[2, 0] != -1):
        B1 = -np.arcsin(R[2, 0])
        B2 = np.pi - B1 # Originally + B1 11/11/23
        A1 = np.arctan2(R[2, 1] / np.cos(B1), R[2, 2] / np.cos(B1))
        A2 = np.arctan2(R[2, 1] / np.cos(B2), R[2, 2] / np.cos(B2))
        C1 = np.arctan2(R[1, 0] / np.cos(B1), R[0, 0] / np.cos(B1))
        C2 = np.arctan2(R[1, 0] / np.cos(B2), R[0, 0] / np.cos(B2))
        return [float(A1), float(B1), float(C1)]
    else:
        C = 0
        if (R[2, 0] == -1):
            B = np.pi / 2.0
            A = C + np.arctan2(R[0, 1], R[0, 2])
        else:
            B = -np.pi / 2.0
            A = -C + np.arctan2(-R[0, 1], -R[0, 2])
        return [float(C1), float(B1), float(A1)]

def rotx(q):
  sq, cq = sin(q), cos(q)
  r = Matrix([
    [1., 0., 0.],
    [0., cq,-sq],
    [0., sq, cq]])
  return r

def roty(q):
  sq, cq = sin(q), cos(q)
  r = Matrix([
    [ cq, 0., sq],
    [ 0., 1., 0.],
    [-sq, 0., cq]])
  return r

def rotz(q):
  sq, cq = sin(q), cos(q)
  r = Matrix([
    [cq,-sq, 0.],
    [sq, cq, 0.],
    [0., 0., 1.]])
  return r

def pose(theta, alpha, a, d):
  # returns the pose T of one joint frame i with respect to the previous joint frame (i - 1). Also known
  # as "modified DH parameters" - https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
  #
  # Parameters:
  # -----------
  #   theta : (num) theta[i]
  #   alpha : (num) alpha[i-1]
  #   a     : (num) a[i-1]
  #   d     : (num) d[i]

  r11, r12 = cos(theta), -sin(theta)
  r23, r33 = -sin(alpha), cos(alpha)
  r21 = sin(theta) * cos(alpha)
  r22 = cos(theta) * cos(alpha)
  r31 = sin(theta) * sin(alpha)
  r32 = cos(theta) * sin(alpha)
  y = -d * sin(alpha)
  z = d * cos(alpha)

  T = Matrix([
    [r11, r12, 0.0, a],
    [r21, r22, r23, y],
    [r31, r32, r33, z],
    [0.0, 0.0, 0.0, 1]
  ])

  T = simplify(T)

  return T


q1, q2, q3, q4, q5, q6= symbols('q1:7')

d90 = pi / 2

#             theta, alpha,      a,     d
T01 = pose(      q1,     0,      0,  0.75)
T12 = pose(q2 - d90,  -d90,   0.35,     0)
T23 = pose(      q3,     0,   1.25,     0)
T34 = pose(      q4,  -d90, -0.054,   1.5)
T45 = pose(      q5,   d90,      0,     0)
T56 = pose(      q6,  -d90,      0,     0)
T6g = pose(       0,     0,      0, 0.303)

# Forward kinematic calculation
T0g_a = simplify(T01 * T12 * T23 * T34 * T45 * T56 * T6g)

Ta = T0g_a.evalf(subs = {
  q1: 1.01249809363771,
  q2: -0.275800363737724,
  q3: -0.115686651053751,
  q4: 1.63446527240323,
  q5: 1.52050002599430,
  q6: -0.815781306199679
})

print("\nT0_a")
print(T0g_a)
print("\nTa")
print(Ta)

# Inverse kinematic calculation, starting with goal pose

px, py, pz = symbols('px py pz', real = True)# position x (px), y (py), z (pz)
alpha, beta, gamma = symbols('alpha beta gamma', real = True)# yaw (alpha), pitch (beta), roll (beta)


# orientation is in gripper frame, change with respect to urdf frame, 
R = rotz(alpha) * roty(beta) * rotx(gamma) * (rotz(pi) * roty(-pi/2)).T
print(R)
T0g_b = Matrix([
  [R[0, 0], R[0, 1], R[0, 2], px],
  [R[1, 0], R[1, 1], R[1, 2], py],
  [R[2, 0], R[2, 1], R[2, 2], pz],
  [0, 0, 0, 1]
])
T0g_b = simplify(trigsimp(T0g_b))
print(T0g_b)

'''
px, py, pz = 0.49792, 1.3673, 2.4988
roll, pitch, yaw = 0.366, -0.078, 2.561

q1: 1.01249809363771
q2: -0.275800363737724
q3: -0.115686651053751
q4: 1.63446527240323
q5: 1.52050002599430
q6: -0.815781306199679
'''
# get the pose (homogenous transforms) of each joint wrt to previous joint
Tb = T0g_b.evalf(subs = {
  gamma: 0.366, #roll
  beta: -0.078, #pitch
  alpha: 2.561, #yaw
  px: 0.49792,
  py: 1.3673,
  pz: 2.4988
})

print("\nT0g_b")
print(T0g_b)
print("\nTb")
print(Tb)

# Fantastic, Ta and Tb are the same, but I still want to check that the forward kinematic transform gives me the 
# same position and orientation prompted from Tb. It won't but I want to see what the difference is.
Tc = np.array([[0.257143295038827,  0.488872082559650,   -0.8335954730625430,   0.49792],
               [0.259329420712765,  0.796053601157403,    0.5468518223770600,   1.36730],
               [0.930927267496960, -0.356795110642117,    0.0779209320563015,   2.49880],
               [             0.00,               0.00,                  0.00,   1.00000]])

print("Tc")
print(Tc)
print("\neuler angles:")
euler_Tc = rotationMatrixToEulerAngles(Tc[:3,:3])
print(euler_Tc)
# x: 0.7896309,  y:  -1.1969437, z: -1.355781    in radians
# x: 45.2425195, y: -68.5798248, z: -77.6805304  in degrees

# So the position is the same as expected (0.49792, 1.3673, 2.4988) but not the rotation matrix.
# Need to add the rotation transform to put it in the tool frame
# Can also check math with Verified with https://danceswithcode.net/engineeringnotes/rotations_in_3d/demo3D/rotations_in_3d_tool.html

print("R0g before adjustment")
print(Tc[:3,:3])


temp2 = np.dot(ymatrix(np.pi/2), zmatrix(-np.pi))
Rcu = np.dot(Tc[:3,:3],temp2)
print("R0g after adjustmnet")
print(Rcu)
euler_Tcu = rotationMatrixToEulerAngles(Rcu)

print("\nPredicted Final orientation:")
print([0.366,-0.078,2.561])
print("Actual")
print(euler_Tcu)

# Seems like the final results for the euler angles matches up

print("T individual")
print("T01")
print(T01.evalf(subs = {q1: 1.01249809363771, q2: -0.275800363737724, q3: -0.115686651053751,
                        q4: 1.63446527240323, q5: 1.52050002599430, q6: -0.815781306199679}))
print("T12")
print(T12.evalf(subs = {q1: 1.01249809363771, q2: -0.275800363737724, q3: -0.115686651053751,
                        q4: 1.63446527240323, q5: 1.52050002599430, q6: -0.815781306199679}))
print("T23")
print(T23.evalf(subs = {q1: 1.01249809363771, q2: -0.275800363737724, q3: -0.115686651053751,
                        q4: 1.63446527240323, q5: 1.52050002599430, q6: -0.815781306199679}))                        
print("T34")
print(T34.evalf(subs = {q1: 1.01249809363771, q2: -0.275800363737724, q3: -0.115686651053751,
                        q4: 1.63446527240323, q5: 1.52050002599430, q6: -0.815781306199679}))
print("T45")
print(T45.evalf(subs = {q1: 1.01249809363771, q2: -0.275800363737724, q3: -0.115686651053751,
                        q4: 1.63446527240323, q5: 1.52050002599430, q6: -0.815781306199679}))
print("T56")
print(T56.evalf(subs = {q1: 1.01249809363771, q2: -0.275800363737724, q3: -0.115686651053751,
                        q4: 1.63446527240323, q5: 1.52050002599430, q6: -0.815781306199679}))
print("T6g")
print(T6g.evalf(subs = {q1: 1.01249809363771, q2: -0.275800363737724, q3: -0.115686651053751,
                        q4: 1.63446527240323, q5: 1.52050002599430, q6: -0.815781306199679}))

T01 = simplify(T01)
T02 = simplify(T01 * T12)
T03 = simplify(T01 * T12 * T23)
print("T01")
print(T01.evalf(subs = {
  q1: 1.01249809363771,
  q2: -0.275800363737724,
  q3: -0.115686651053751,
}))
print("T02")
print(T02.evalf(subs = {
  q1: 1.01249809363771,
  q2: -0.275800363737724,
  q3: -0.115686651053751,
}))
print("T03")
print(T03.evalf(subs = {
  q1: 1.01249809363771,
  q2: -0.275800363737724,
  q3: -0.115686651053751,
}))


# Mini arm parameters
#T01 = pose(q1,        0,      0, 0.095)
#T12 = pose(q2-d90, -d90,  0.015,     0)
#T23 = pose(q3,        0,  0.120,     0)
#T34 = pose(q4,     -d90,    0.0,  0.09)
#T45 = pose(q5,      d90,      0,     0)
#T56 = pose(q6,     -d90,      0,     0)
#T6g = pose(0,         0,      0, 0.050)

                         
#T0g_a = simplify(T01 * T12 * T23 * T34 * T45 * T56 * T6g)
#Ta = T0g_a.evalf(subs ={q1: 0.000,q2: 0.000,q3: 0.000,q4: 0.000,q5: 0.000,q6: 0.000})
#print("\nT01")
#print(T01.evalf(subs = {q1: 0.000,q2: 0.000,q3: 0.000,q4: 0.000,q5: 0.000,q6: 0.000}))
#print("\nT02")
#print(simplify(T01 * T12).evalf(subs = {q1: 0.000,q2: 0.000,q3: 0.000,q4: 0.000,q5: 0.000,q6: 0.000}))
#print("\nminiarm transform")
#print(Ta)
#Tc = np.array([[   0,     0,   1.00,    0.155], 
#               [   0, -1.00,      0,        0], 
#               [1.00,     0,      0,    0.215], 
#               [   0,     0,      0,    1.00]])
               
#print("\nFinal orientation:")
#print([0.000, 0.000, 0.000])
#temp2 = np.dot(ymatrix(np.pi/2), zmatrix(-np.pi))
#Rcu = np.dot(Tc[:3,:3],temp2)
#euler_Tcu = rotationMatrixToEulerAngles(Rcu)
#print(euler_Tcu)