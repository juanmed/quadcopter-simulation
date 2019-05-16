"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
import scipy.integrate as integrate
from utils.quaternion import Quaternion
#from utils.utils import RPYToRot, RotToQuat, RotToRPY
from utils.utils import RotToQuat
from utils.utils import RPYToRot_ZYX as RPYToRot
from utils.utils import RotToRPY_ZYX as RotToRPY
import model.params as params

class Quadcopter:
    """ Quadcopter class

    state  - 1 dimensional vector but used as 13 x 1. [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
             where [qw, qx, qy, qz] is quternion and [p, q, r] are angular velocity [roll_dot, pitch_dot, yaw_dot]
    F      - 1 x 1, thrust output from controller
    M      - 3 x 1, moments output from controller
    params - system parameters struct, arm_length, g, mass, etc.
    """

    def __init__(self, pos, attitude):
        """ pos = [x,y,z] attitude = [rool,pitch,yaw]
            """
        self.state = np.zeros(13)
        roll, pitch, yaw = attitude
        rot    = RPYToRot(roll, pitch, yaw)
        quat   = RotToQuat(rot)
        self.state[0] = pos[0]
        self.state[1] = pos[1]
        self.state[2] = pos[2]
        self.state[6] = quat[0]
        self.state[7] = quat[1]
        self.state[8] = quat[2]
        self.state[9] = quat[3]

    def world_frame(self):
        """ position returns a 3x6 matrix
            where row is [x, y, z] column is m1 m2 m3 m4 origin h
            """
        origin = self.state[0:3]
        quat = Quaternion(self.state[6:10])
        rot = quat.as_rotation_matrix()
        wHb = np.r_[np.c_[rot,origin], np.array([[0, 0, 0, 1]])]
        quadBodyFrame = params.body_frame.T
        quadWorldFrame = wHb.dot(quadBodyFrame)
        world_frame = quadWorldFrame[0:3]
        return world_frame

    def position(self):
        return self.state[0:3]

    def velocity(self):
        return self.state[3:6]

    def attitude(self):
        rot = Quaternion(self.state[6:10]).as_rotation_matrix()
        return RotToRPY(rot)

    def omega(self):
        return self.state[10:13]

    def rotor_drag(self, v, vw, w, Fi, wRb, di):
        """
        Compute drag force for a single rotor with thrust Fi
        """
        va = v - vw # relative air velocity w.r.t. center of gravity
        Va_i = np.dot(wRb.T,va) + np.cross(w,di, axis = 0) # relative air velocity w.r.t each rotor
        Fa_i = -1.0*np.sqrt(Fi)*np.dot(params.D,Va_i)
        return Fa_i

    def total_rotor_drag(self, v, vw, w, Fi_s, wRb):
        """
        Compute sum of drag force from all rotors
        """
        rotor_arm_vectors = [params.r1, params.r2, params.r3, params.r4]
        rotor_thrusts = [Fi_s[0][0], Fi_s[1][0], Fi_s[2][0], Fi_s[3][0]]
        Fa = 0
        for arm, thrust in zip(rotor_arm_vectors, rotor_thrusts):
            Fa = Fa + self.rotor_drag(v, vw, w, thrust, wRb, arm)

        return Fa



    def state_dot(self, state, t, F, M, Fi_s):
        x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r = state
        quat = np.array([qw,qx,qy,qz])

        bRw = Quaternion(quat).as_rotation_matrix() # world to body rotation matrix
        wRb = bRw.T # orthogonal matrix inverse = transpose

        # acceleration - Newton's second law of motion
        #accel = (1.0 / params.mass) * (wRb.dot(np.array([[0, 0, F]]).T) - np.array([[0, 0, params.mass * params.g]]).T)
                    

        v = np.array([[xdot],[ydot],[zdot]])
        vw = np.array([[0],[0],[0]])                # wind velocity
        omega = np.array([[p],[q],[r]])
        Fa =  self.total_rotor_drag(v, vw, omega, Fi_s, wRb)   # total rotor drag in body frame
        accel = -params.g*params.e3 + (F/params.mass)*np.dot(wRb,params.e3) + np.dot(wRb, Fa)

        # angular velocity - using quternion
        # http://www.euclideanspace.com/physics/kinematics/angularvelocity/
        K_quat = 2.0; # this enforces the magnitude 1 constraint for the quaternion
        quaterror = 1.0 - (qw**2 + qx**2 + qy**2 + qz**2)
        qdot = (-1.0/2) * np.array([[0, -p, -q, -r],
                                    [p,  0, -r,  q],
                                    [q,  r,  0, -p],
                                    [r, -q,  p,  0]]).dot(quat) + K_quat * quaterror * quat;

        # angular acceleration - Euler's equation of motion
        # https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
        omega = np.array([p,q,r])
        pqrdot = params.invI.dot( M.flatten() - np.cross(omega, params.I.dot(omega)) )
        state_dot = np.zeros(13)
        state_dot[0]  = xdot
        state_dot[1]  = ydot
        state_dot[2]  = zdot
        state_dot[3]  = accel[0][0]
        state_dot[4]  = accel[1][0]
        state_dot[5]  = accel[2][0]
        state_dot[6]  = qdot[0]
        state_dot[7]  = qdot[1]
        state_dot[8]  = qdot[2]
        state_dot[9]  = qdot[3]
        state_dot[10] = pqrdot[0]
        state_dot[11] = pqrdot[1]
        state_dot[12] = pqrdot[2]

        return state_dot

    def update(self, dt, F, M):
        # limit thrust and Moment
        L = params.arm_length
        r = params.r
        prop_thrusts = params.invA.dot(np.r_[np.array([[F]]), M])
        prop_thrusts_clamped = np.maximum(np.minimum(prop_thrusts, params.maxF/params.number_of_rotors), params.minF/params.number_of_rotors)
        F = np.sum(prop_thrusts_clamped)
        M = params.A[1:].dot(prop_thrusts_clamped)
        self.state = integrate.odeint(self.state_dot, self.state, [0,dt], args = (F, M, prop_thrusts_clamped))[1]
