# Set of functions that generate the states and inputs of the quadrotor
# given 4 flat outputs, namely: position (3-vector) and yaw angle.
#


import numpy as np
import model.params as params

def RotToRPY_ZYX(R):
    """
        Euler angle convention is ZYX, which means first apply
        rotaion of psi-degrees around Z axis, then rotation of
        theta-degrees around new Y axis, and then rotation of 
        phi-degrees around new X axis.

        ** The rotation R received should be from body to world frame. **
    """
    theta = np.arcsin(-1.0*R[2][0])
    phi = np.arctan2(R[2][1]/np.cos(theta), R[2][2]/np.cos(theta))
    psi = np.arctan2(R[1][0]/np.cos(theta), R[0][0]/np.cos(theta))

    return np.array([ [phi], [theta], [psi] ]) 

def RotToRPY_ZXY(R):
    """
    phi, theta, psi = roll, pitch , yaw
    The euler angle convention used is ZXY. This means: first a rotation of psi-degrees
    around Z axis, then rotation of phi-degress around X axis, and finally rotation of
    theta-degrees around Y axis

        ** The rotation R received should be from world to body frame **
    """
    phi = np.arcsin(R[1,2])
    theta = np.arctan2(-R[0,2]/np.cos(phi),R[2,2]/np.cos(phi))
    psi = np.arctan2(-R[1,0]/np.cos(phi),R[1,1]/np.cos(phi))
    return np.array([ [phi], [theta], [psi] ])


def get_x(sigma1):
    return sigma1

def get_y(sigma2):
    return sigma2

def get_z(sigma3):
    return sigma3

def get_psi(sigma4):
    return sigma4

def get_zb(acc, z_w, vel):
    """
    The body-axis Z vector is in the direction of the thrust vector
    but with magnitude = 1
    """
    inertial_vector = params.mass*acc
    weight_vector = params.mass*params.g*z_w
    drag_vector = params.gamma*vel
    thrust_vector = inertial_vector + weight_vector + drag_vector
    return thrust_vector/np.linalg.norm(thrust_vector)

def get_real_thrust(z_b, acc, z_w, vel):
    """
    Calculate real thrust magnitude
    """
    inertial_vector = params.mass*acc
    weight_vector = params.mass*params.g*z_w
    drag_vector = params.gamma*vel   
    real_thrust = np.dot(z_b.T, inertial_vector + weight_vector + drag_vector) 
    return real_thrust[0][0] # return scalar only

def get_real_thrust_dot(z_b, jerk, acc):
    inertial_vector_zaxis_dot = params.mass*np.dot(z_b.T, jerk)
    drag_vector_zaxis_dot = params.gamma*np.dot(z_b.T, acc)
    real_thrust_dot = inertial_vector_zaxis_dot + drag_vector_zaxis_dot
    return real_thrust_dot[0][0] #return scalar only

def get_command_thrust_positive_zb(T_real, vel, z_b):
    """
        Return command thrust for the quadrotor. Assumes thrust direction is in +z_b axis
    """
    command_thrust = T_real - params.gamma*np.dot(vel.T, z_b)
    return command_thrust[0][0] # return scalar only

def get_xc(sigma4):
    x_c = np.array([[np.cos(sigma4)],[np.sin(sigma4)],[0.0]])
    return x_c

def get_yc(sigma4):
    y_c = np.array([[-1.0*np.sin(sigma4)],[np.cos(sigma4)],[0.0]])
    return y_c  

def get_xb(y_c,z_b):
    a = np.cross(y_c, z_b, axis = 0)
    return a/np.linalg.norm(a)

def get_yb(z_b,x_b):
    a = np.cross(z_b,x_b, axis = 0)
    return a/np.linalg.norm(a)

def get_wx(y_b,jerk,T_real, acc):
    """ Return body x axis angular velocity"""
    a = -1.0*params.mass*np.dot(y_b.T,jerk)
    b = -1.0*params.gamma*np.dot(y_b.T, acc)
    w_x = (a+b)/T_real
    return w_x[0][0] # return scalar only

def get_wy(x_b, jerk, T_real, acc):
    """ Return body y axis angular velocity"""

    a = params.mass*np.dot(x_b.T,jerk)
    b = params.gamma*np.dot(x_b.T,acc)
    w_y = (a+b)/T_real
    return w_y[0][0] # return scalar only

def get_wz(psi_rate,x_c,x_b,w_y,y_c,z_b):
    """
        Will compute as wz = (a + b)/c
    """
    a = psi_rate*np.dot(x_c.T,x_b)
    b = w_y*np.dot(y_c.T,z_b)
    c = np.linalg.norm(np.cross(y_c,z_b,axis = 0))
    w_z = (a+b)/c
    return w_z[0][0] # return scalar only

def get_wy_dot(x_b,s,T_real_dot,w_y,T_real,w_x,w_z,jerk):
    """
        Will use wy_dot = (a + b + c+ d)/e
    """
    a = params.mass*np.dot(x_b.T,s) 
    b = -2.0*T_real_dot*w_y
    c = -1.0*T_real*w_x*w_z
    d = params.gamma*np.dot(x_b.T,jerk)
    e = T_real
    w_y_dot = (a+b+c+d)/e
    return w_y_dot[0][0] # return scalar only

def get_wx_dot(y_b,s,T_real_dot,w_x,T_real,w_y,w_z,jerk):

    """
        Will use wx_dot = (a + b + c +d)/e
    """
    a = -1.0*params.mass*np.dot(y_b.T,s)
    b = -2.0*T_real_dot*w_x
    c = T_real*w_y*w_z
    d = -1.0*params.gamma*np.dot(y_b.T, jerk)
    e = T_real
    w_x_dot = (a+b+c+d)/e
    return w_x_dot[0][0] # return scalar only

def get_wz_dot(psi_acc,x_c,x_b,psi_rate,w_z,y_b,w_y,z_b,w_x,y_c,w_y_dot):
    """
        Will compute as w_z_dot = (a+b+c+d+e+f)/g

    """
    a = psi_acc*np.dot(x_c.T,x_b)
    b = 2.0*psi_rate*w_z*np.dot(x_c.T,y_b)
    c = -2.0*psi_rate*w_y*np.dot(x_c.T,z_b)
    d = -1.0*w_x*w_y*np.dot(y_c.T,y_b)
    e = -1.0*w_x*w_z*np.dot(y_c.T,z_b)
    f = w_y_dot*np.dot(y_c.T,z_b)
    g = np.linalg.norm(np.cross(y_c,z_b,axis = 0))
    w_z_dot = (a+b+c+d+e+f)/g
    return w_z_dot[0][0] # return scalar only

# This correspond to [u1, u2, u3]
def get_ux(w_dot_,w_):
    u_x = np.dot(params.I,w_dot_) + np.cross(w_, np.dot(params.I,w_), axis = 0)
    return u_x

def get_ua(T_command,z_b, Rbw, vel):
    """
        ua = -g*z_w +u1*z_b/m
    """

    normalized_gravity = -1.0*params.g*params.e3 
    normalized_command_thrust = T_command*z_b/params.mass
    
    pi_e3 = np.diag([1.0,1.0,1.0]) - np.dot(params.e3, params.e3.T)
    D = params.gamma*pi_e3
    bodyframe_v = np.dot(Rbw.T,vel)
    bodyframe_drag = np.dot(D,bodyframe_v)
    worldframe_drag = np.dot(Rbw, bodyframe_drag)
    normalized_worldframe_drag = -1.0*worldframe_drag/params.mass
    u_a = normalized_gravity + normalized_command_thrust + normalized_worldframe_drag
    return u_a

def get_ub(w_,M):
    u_b = np.dot(params.invI,(-1.0*np.cross(w_, np.dot(params.I,w_), axis = 0) + M))
    return u_b

def get_uc(w_,ori):
    """
    """
    phi_ = ori[0][0]
    theta_ = ori[1][0]
    psi_ = ori[2][0]

    peta = np.array([
        [1.0, np.sin(phi_)*np.tan(theta_), np.cos(phi_)*np.tan(theta_)],
        [0.0, np.cos(phi_),-1.0*np.sin(phi_)],
        [0.0, np.sin(phi_)/np.cos(theta_), np.cos(phi_)/np.cos(theta_)]])
    u_c = np.dot(peta,w_)

    return u_c 

def compute_ref(trajectory):
    """
        Compute all reference states and inputs from the given desired trajectory point using 
        differential flatness property.
    """
    # first convert all input np.array to np.matrices to simplify 
    # computation.
    # This should be changed to use np.arrays only as np.matrix is not recommended anymore

    # extract all quantities from given trajectory
    pos_traj = trajectory[0].reshape(3,1) # 3-vector
    vel_traj = trajectory[1].reshape(3,1) # 3-vector
    acc_traj = trajectory[2].reshape(3,1) # 3-vector
    jerk_traj = trajectory[3].reshape(3,1) # 3-vector
    snap_traj = trajectory[4].reshape(3,1) # 3-vector
    yaw_traj = trajectory[5] # scalar
    yaw_dot_traj = trajectory[6] # scalar
    yaw_ddot_traj = trajectory[7] # scalar
    """
    print("pos_traj: {}\n".format(pos_traj)+
          "vel_traj: {}\n".format(vel_traj)+
          "acc_traj: {}\n".format(acc_traj)+
          "jerk_traj: {}\n".format(jerk_traj)+
          "snap_traj: {}\n".format(snap_traj)+
          "yaw_traj: {}\n".format(yaw_traj)+
          "yaw_dot_traj: {}\n".format(yaw_dot_traj)+
          "yaw_ddot_traj: {}\n".format(yaw_ddot_traj))
    """
    # ---------------------------------------- # 
    #  Calculate all states using flat output  #
    #  trajectory                              #
    # ---------------------------------------- #

    # Orientation
    z_b = get_zb(acc_traj, params.e3, vel_traj)
    x_c = get_xc(yaw_traj) 
    y_c = get_yc(yaw_traj)
    x_b = get_xb(y_c,z_b) 
    y_b = get_yb(z_b,x_b) 
    """
    print("z_b: {}\n".format(z_b)+
          "y_b: {}\n".format(y_b)+
          "x_b: {}\n".format(x_b))
    """
    #print("gamma: {}".format(params.gamma))
    # Real Thrust 
    T_real = get_real_thrust(z_b, acc_traj, params.e3, vel_traj)
    T_real_dot = get_real_thrust_dot(z_b, jerk_traj, acc_traj)
    T_command = get_command_thrust_positive_zb(T_real, vel_traj, z_b)

    # Angular Velocities
    w_x = get_wx(y_b, jerk_traj, T_real, acc_traj) 
    w_y = get_wy(x_b, jerk_traj, T_real, acc_traj) 
    w_z = get_wz(yaw_dot_traj, x_c, x_b, w_y, y_c, z_b) 

    # Angular accelerations
    w_y_dot = get_wy_dot(x_b, snap_traj, T_real_dot, w_y, T_real, w_x, w_z, jerk_traj) 
    w_x_dot = get_wx_dot(y_b, snap_traj, T_real_dot, w_x, T_real, w_y, w_z, jerk_traj) 
    w_z_dot = get_wz_dot(yaw_ddot_traj,x_c, x_b, yaw_dot_traj, w_z, y_b, w_y, z_b, w_x, y_c, w_y_dot)
             
    
    # make angular acceleration vector w_dot
    # remember each element is a 1x1 matrix so have to extract that element...
    w_dot_ = np.array([[w_x_dot],[w_y_dot],[w_z_dot]]) 

    # make angular velocity vector w
    w_ = np.array([[w_x],[w_y],[w_z]])

    # get vector of torque inputs u2, u3, u4
    u_x = get_ux(w_dot_, w_) # get_ux(w_dot_,w_)

    # get rotation matrix from base frame to world frame
    # for current desired trajectory point.
    # This matrix represents the orientation of the quadrotor
    Rbw = np.concatenate((x_b, y_b, z_b), axis = 1)

    # Get roll pitch yaw angles assuming ZXY Euler angle convention
    # This means: first rotate psi degrees around Z axis,
    # then theta degrees around Y axis, and lastly phi degrees around X axis
    
    or_ = RotToRPY_ZYX(Rbw)  # assuming ZYX Eugler angle convention, so sent matrix should be 
                              # body to world frame

    # compute u_a input for system reference
    # can be computed as follows or simply the received acc_traj
    # vector after conversion to matrix. Both are exactly the same quantity
    u_a = get_ua(T_command,z_b, Rbw, vel_traj) # get_ua(u_1,z_b)

    # compute u_b input for system reference
    u_b = get_ub(w_, u_x) # get_ub(w_,M)

    # compute u_c input for system reference
    u_c = get_uc(w_,or_) #  get_uc(w_,ori)

    # we need to return back the 1) reference state vector and 2) reference inputs
    # The reference state vector is : (x,y,z, v_x, v_y, v_z, phi, theta, psi, p, q, r)
    # where x,y,z: are position coordinates
    # v_x, v_y, v_z: are velocities
    # phi, theta, psi: are orientation euler angles as described above
    # p, q, r: are angular velocities in body frame X,Y,Z axis respectively

    # we send the received pos_traj, and vel_traj vectors as the reference pos and vel vectors
    # because that is the result from the differential flatness output selection
    #return [pos_traj.T, vel_traj.T, or_, w_, acc_traj.T, w_dot_, R_, u_c, u_1, u_x]
    return [pos_traj, vel_traj, or_, w_, w_dot_, u_a, u_b, u_c, T_command, u_x, Rbw, acc_traj, jerk_traj, snap_traj, yaw_traj, yaw_dot_traj, yaw_ddot_traj]




