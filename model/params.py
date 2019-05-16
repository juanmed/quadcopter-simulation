"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np

mass = 0.18 # kg
g = 9.81 # m/s/s
I = np.array([(0.00025, 0, 2.55e-6),
              (0, 0.000232, 0),
              (2.55e-6, 0, 0.0003738)]);

invI = np.linalg.inv(I)
arm_length = 0.086 # meter
height = 0.05
minF = 0.0
maxF = 2.0 * mass * g
L = arm_length
H = height
km = 1.5e-9   # torque constant
kf = 6.11e-8  # thrust constant
r = km / kf

#  [ F  ]         [ F1 ]
#  | M1 |  = A *  | F2 |
#  | M2 |         | F3 |
#  [ M3 ]         [ F4 ]
A = np.array([[ 1,  1,  1,  1],
              [ 0,  L,  0, -L],
              [-L,  0,  L,  0],
              [ r, -r,  r, -r]])

B = np.array([[kf, kf, kf, kf],
              [0, L*kf, 0, -L*kf],
              [-L*kf, 0, L*kf, 0],
              [-km, km, -km, km]])

invA = np.linalg.inv(A)
invB = np.linalg.inv(B)

body_frame = np.array([(L, 0, 0, 1),
                       (0, L, 0, 1),
                       (-L, 0, 0, 1),
                       (0, -L, 0, 1),
                       (0, 0, 0, 1),
                       (0, 0, H, 1)])

# R3 basis vectors
e1 = np.array([[1.0],[0.0],[0.0]])
e2 = np.array([[0.0],[1.0],[0.0]])
e3 = np.array([[0.0],[0.0],[1.0]])


# rotor drag parameters (rigid propellers)
cd1 = 0.055 # see [1]
gamma = 2*np.sqrt(mass*g)*cd1

# References:
#[1] Kai, J. M., Allibert, G., Hua, M. D., & Hamel, T. (2017). 
# Nonlinear feedback control of Quadrotors exploiting First-Order Drag Effects. 
# IFAC-PapersOnLine, 50(1), 8189-8195. 
# https://doi.org/10.1016/j.ifacol.2017.08.1267