# Set of functions that generate the states and inputs of the quadrotor
# given 4 flat outputs, namely: position (3-vector) and yaw angle.
#


import numpy as np
#import model.params # import params

#from math import sin, cos, asin, atan2, sqrt

def RotToRPY(R):
	"""
		Euler angle convention is ZYX, which means first apply
		rotaion of psi-degrees around Z axis, then rotation of
		theta-degrees around new Y axis, and then rotation of 
		phi-degrees around new X axis.

		The rotation R received should be from body to world frame.
	"""
	theta = np.arcsin(-1.0*R.item(2,0))
	phi = np.arctan2(R.item(2,1)/np.cos(theta),R.item(2,2)/np.cos(theta))
	psi = np.arctan2(R.item(1,0)/np.cos(theta),R.item(0,0)/np.cos(theta))

	return np.matrix([ [phi], [theta], [psi] ]) 

def get_x(sigma1):
	return sigma1

def get_y(sigma2):
	return sigma2

def get_z(sigma3):
	return sigma3

def get_psi(sigma4):
	return sigma4

def get_u1(t):
	u1 = m*np.linalg.norm(t)
	return u1

def get_zb(t):
	zb = t/(np.linalg.norm(t))
	return zb

def get_u1_dot(z_b,j):
	u1_dot = m*z_b.T*j
	return u1_dot

def get_t_vector(sigma1, sigma2,sigma3):
	# construct the t vector at each point in time
	t_vec = np.matrix([[sigma1],[sigma2],[sigma3+g]])
	return t_vec

def get_xc(sigma4):
	x_c = np.matrix([[np.cos(sigma4)],[np.sin(sigma4)],[0.0]])
	return x_c

def get_yc(sigma4):
	y_c = np.matrix([[-1.0*np.sin(sigma4)],[np.cos(sigma4)],[0.0]])
	return y_c	

def get_xb(y_c,z_b):
	a = np.cross(y_c, z_b, axis = 0)
	a = np.matrix(a)
	return a/np.linalg.norm(a)

def get_yb(z_b,x_b):
	a = np.cross(z_b,x_b, axis = 0)
	a = np.matrix(a)
	return a/np.linalg.norm(a)

def get_wx(y_b,j,u_1):
	wx = -1.0*m*((y_b.T)*j)/u_1
	return wx

def get_wy(x_b,j,u_1):
	wy = m*((x_b.T)*j)/u_1
	return wy

def get_wz(psi_rate,x_c,x_b,w_y,y_c,z_b):
	"""
		Will compute as wz = (a + b)/c
	"""
	a = psi_rate*(x_c.T)*x_b
	b = w_y*(y_c.T)*z_b
	c = np.linalg.norm(np.cross(y_c,z_b,axis = 0))
	wz = (a+b)/c
	return wz

def get_wy_dot(x_b,s,u_1_dot,w_y,u_1,w_x,w_z):
	"""
		Will use wy_dot = (a + b + c)/d
	"""
	a = (x_b.T)*s 
	b = -2.0*u_1_dot*w_y/m
	c = -1.0*u_1*w_x*w_z/m
	d = u_1/m
	w_y_dot = (a+b+c)/d
	return w_y_dot

def get_wx_dot(y_b,s,u_1_dot,w_x,u_1,w_y,w_z):

	"""
		Will use wx_dot = (a + b + c)/d
	"""
	a = (y_b.T)*s
	b = 2.0*u_1_dot*w_x/m
	c = -1.0*u_1*w_y*w_z/m
	d = u_1/m
	w_x_dot = (a+b+c)/d
	return w_x_dot

def get_wz_dot(psi_acc,x_c,x_b,psi_rate,w_z,y_b,w_y,z_b,w_x,y_c,w_y_dot):
	"""
		Will compute as w_z_dot = (a+b+c+d+e+f)/g

	"""
	a = psi_acc*(x_c.T)*x_b
	b = 2.0*psi_rate*w_z*(x_c.T)*y_b
	c = -2.0*psi_rate*w_y*(x_c.T)*z_b
	d = -1.0*w_x*w_y*(y_c.T)*z_b
	e = -1.0*w_x*w_z*(y_c.T)*z_b
	f = w_y_dot*(y_c.T)*z_b
	g = np.linalg.norm(np.cross(y_c,z_b,axis = 0))
	w_z_dot = (a+b+c+d+e+f)/g
	#print("w_z_dot type is: {}".format(type(w_z_dot)))
	return w_z_dot

# This correspond to [u1, u2, u3]
def get_ux(w_dot_,w_):
	u_x = I*w_dot_ + np.matrix(np.cross(w_, I*w_, axis = 0))
	return u_x

def get_ua(u_1,z_b):
	"""
		ua = -g*z_w +u1*z_b/m
	"""
	u_a = -g*np.matrix([[0.0],[0.0],[1.0]]) + u_1*z_b/m
	return u_a

def get_ub(w_,M):
	u_b = invI*(-1.0*np.cross(w_, I*w_, axis = 0) + M)
	return u_b

def get_uc(w_,ori):
	"""
	"""
	phi_ = ori.item(0)
	theta_ = ori.item(1)
	psi_ = ori.item(2)

	peta = np.matrix([
		[1.0, np.sin(phi_)*np.tan(theta_), np.cos(phi_)*np.tan(theta_)],
		[0.0, np.cos(phi_),-1.0*np.sin(phi_)],
		[0.0, np.sin(phi_)/np.cos(theta_), np.cos(phi_)/np.cos(theta_)]])
	u_c = peta*w_

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
	pos_traj = trajectory[0] 
	vel_traj = trajectory[1]
	acc_traj = trajectory[2]
	jerk_traj = trajectory[3]
	snap_traj = trajectory[4]
	yaw_traj = trajectory[5]
	yaw_dot_traj = trajectory[6]
	yaw_ddot_traj = trajectory[7]

	# convert all vectors from np.array to np.matrix for compatibility and
	# ease
	pos_traj = np.matrix(pos_traj)
	vel_traj = np.matrix(vel_traj)
	acc_traj = np.matrix(acc_traj)
	jerk_traj = np.matrix(jerk_traj)
	snap_traj = np.matrix(snap_traj)
	yaw_traj = np.matrix(yaw_traj)
	yaw_dot_traj = np.matrix(yaw_dot_traj)
	yaw_ddot_traj = np.matrix(yaw_ddot_traj)

	t_vec = get_t_vector(acc_traj.item(0), acc_traj.item(1), acc_traj.item(2))

	u_1 = get_u1(t_vec)
	z_b = get_zb(t_vec)
	y_c = get_yc(yaw_traj.item(0))
	x_b = get_xb(y_c,z_b)
	y_b = get_yb(z_b,x_b)
	j_ = np.matrix([[jerk_traj.item(0)],[jerk_traj.item(1)],[jerk_traj.item(2)]])
	w_x = get_wx(y_b,j_,u_1)
	w_y = get_wy(x_b,j_,u_1)
	x_c = get_xc(yaw_traj.item(0))
	w_z = get_wz(yaw_dot_traj.item(0),x_c,x_b,w_y,y_c,z_b)
	u_1_dot = get_u1_dot(z_b,j_)
	s_ = np.matrix([[snap_traj.item(0)],[snap_traj.item(1)],[snap_traj.item(2)]])
	w_y_dot = get_wy_dot(x_b,s_,u_1_dot,w_y,u_1,w_x,w_z)
	w_x_dot = get_wx_dot(y_b, s_, u_1_dot, w_x, u_1, w_y, w_z)
	w_z_dot = get_wz_dot(yaw_ddot_traj.item(0),x_c, x_b, yaw_dot_traj.item(0), w_z, y_b, w_y, z_b, w_x, y_c, w_y_dot)
	
	# make angular acceleration vector w_dot
	# remember each element is a 1x1 matrix so have to extract that element...
	w_dot_ = np.matrix([[w_x_dot.item(0)],[w_y_dot.item(0)],[w_z_dot.item(0)]]) 

	# make angular velocity vector w
	w_ = np.matrix([[w_x.item(0)],[w_y.item(0)],[w_z.item(0)]])

	# get vector of torque inputs u2, u3, u4
	u_x = get_ux(w_dot_, w_)

	# get rotation matrix from base frame to world frame
	# for current desired trajectory point.
	# This matrix represents the orientation of the quadrotor
	R_ = np.concatenate((x_b, y_b, z_b), axis = 1)

	# Get rall pitch yaw angles assuming ZYX Euler angle convention
	# This means: first rotate psi degrees around Z axis,
	# then theta degrees around Y axis, and lastly phi degrees around X axis
	or_ = RotToRPY(R_)

	# compute u_a input for system reference
	# can be computed as follows or simply the received acc_traj
	# vector after conversion to matrix. Both are exactly the same quantity
	u_a = get_ua(u_1, z_b)

	# compute u_b input for system reference
	u_b = get_ub(w_, u_x)

	# compute u_c input for system reference
	u_c = get_uc(w_,or_)

	# we need to return back the 1) reference state vector and 2) reference inputs
	# The reference state vector is : (x,y,z, v_x, v_y, v_z, phi, theta, psi, p, q, r)
	# where x,y,z: are position coordinates
	# v_x, v_y, v_z: are velocities
	# phi, theta, psi: are orientation euler angles as described above
	# p, q, r: are angular velocities in body frame X,Y,Z axis respectively

	# we send the received pos_traj, and vel_traj vectors as the reference pos and vel vectors
	# because that is the result from the differential flatness output selection
	return [pos_traj.T, vel_traj.T, or_, w_, u_a, u_b, u_c, u_1, u_x]



"""
# define constants
g =  params.g #9.81 #m/s2
b = 0.01  # air drag/friction force
#c = 0.2 #air friction constant

# quadrotor physical constants
m = params.m #0.18  #kg  mass of the quadrotor


#I = np.matrix([[0.00025, 0, 2.55e-6],
#              [0, 0.000232, 0],
#              [2.55e-6, 0, 0.0003738]]);

I = np.matrix(params.I)

invI = np.matrix(params.invI)
"""


# define constants
g = 9.81 #m/s2
b = 0.01  # air drag/friction force
#c = 0.2 #air friction constant

# quadrotor physical constants
m = 0.18  #kg  mass of the quadrotor
Ktao =0.02           # Drag torque constant for motors
Kt =0.2             # Thrust constant for motors
I = np.matrix([[0.00025, 0, 2.55e-6],
              [0, 0.000232, 0],
              [2.55e-6, 0, 0.0003738]]);

invI = np.linalg.inv(I)