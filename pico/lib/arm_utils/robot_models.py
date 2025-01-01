import ulab.numpy as np

# Collection of robot models using different parameters. 
# 
# Below I have the kuka arm, parameters primarily used 
# for testing the IK math, and the miniarm models


miniarm_params =        [{'theta':   0.000, 'alpha':    0.000, 'a':   0.000, 'd': 0.095},
                         {'theta':-np.pi/2, 'alpha': -np.pi/2, 'a':   0.015, 'd': 0.000},
                         {'theta':   0.000, 'alpha':    0.000, 'a':   0.120, 'd': 0.000},
                         {'theta':   0.000, 'alpha': -np.pi/2, 'a':   0.000, 'd': 0.090},
                         {'theta':   0.000, 'alpha':  np.pi/2, 'a':   0.000, 'd': 0.000},
                         {'theta':   0.000, 'alpha': -np.pi/2, 'a':   0.000, 'd': 0.000},
                         {'theta':   0.000, 'alpha':    0.000, 'a':   0.000, 'd': 0.030}]

#miniarm_6DOF_params =   [{'d':  0.095, 'a':   0.015, 'alpha': -np.pi/2, 'theta':    0.000},
#                         {'d':  0.000, 'a':   0.120, 'alpha':    0.000, 'theta': 0.},#-np.pi/2},
#                         {'d':  0.000, 'a':   0.000, 'alpha': -np.pi/2, 'theta':    0.000},
#                         {'d':  0.090, 'a':   0.000, 'alpha':  np.pi/2, 'theta':    0.000},
#                         {'d':  0.000, 'a':   0.000, 'alpha': -np.pi/2, 'theta':    0.000},
#                         {'d':  0.030, 'a':   0.000, 'alpha':    0.000, 'theta':    0.000}]
                         
miniarm_6DOF_params =   [{'d':  0.035, 'a':   0.015, 'alpha': -np.pi/2, 'theta':    0.000},
                         {'d':  0.000, 'a':   0.120, 'alpha':    0.000, 'theta': -np.pi/2},
                         {'d':  0.000, 'a':   0.000, 'alpha': -np.pi/2, 'theta':    0.000},
                         {'d':  0.090, 'a':   0.000, 'alpha':  np.pi/2, 'theta':    0.000},
                         {'d':  0.000, 'a':   0.000, 'alpha': -np.pi/2, 'theta':    0.000},
                         {'d':  0.030, 'a':   0.000, 'alpha':    0.000, 'theta':    0.000}]
                          
# KUKA KR210 L150
kuka_params =           [{'theta':       0, 'alpha':        0, 'a':      0, 'd':  0.750},
                         {'theta':-np.pi/2, 'alpha': -np.pi/2, 'a':  0.350, 'd':      0},
                         {'theta':       0, 'alpha':        0, 'a':  1.250, 'd':      0},
                         {'theta':       0, 'alpha': -np.pi/2, 'a': -0.054, 'd':    1.5},
                         {'theta':       0, 'alpha':  np.pi/2, 'a':      0, 'd':      0},
                         {'theta':       0, 'alpha': -np.pi/2, 'a':      0, 'd':      0},
                         {'theta':       0, 'alpha':        0, 'a':      0, 'd':  0.303}]

kuka_6DOF_params =      [{'d':  0.750, 'a':   0.350, 'alpha': -np.pi/2, 'theta':    0.000},
                         {'d':  0.000, 'a':   1.250, 'alpha':    0.000, 'theta': -np.pi/2},
                         {'d':  0.000, 'a':  -0.054, 'alpha': -np.pi/2, 'theta':    0.000},
                         {'d':  1.500, 'a':   0.000, 'alpha':  np.pi/2, 'theta':    0.000},
                         {'d':  0.000, 'a':   0.000, 'alpha': -np.pi/2, 'theta':    0.000},
                         {'d':  0.303, 'a':   0.000, 'alpha':    0.000, 'theta':    0.000}]

					   
        # Set the Mini-Arm transform matrix linkage dimensions in terms of x,y,z in mm         
        #self.robot.a1z = 60 
        #self.robot.a2x = 15
        #self.robot.a2z = 35
        #self.robot.a3z = 120
        #self.robot.a4x = 90 
        #self.robot.a5x = 30 
        #self.robot.a6x = 50 

