#!/usr/bin/env python
#
# This code calculates LQR gains for a drone system whose reference signal is obtained
# by differential flatness. The input to the system is the body frame thrust and body
# frame angular velocities, and output is the world frame position, velocity and 
# angular position given by euler angles.
# 
# For reference please see:
# Differential Flatness Based Control of a Rotorcraft For Aggressive Maneuvers BYU ScholarsArchive Citation Differential Flatness Based Control of a Rotorcraft For
# Aggressive Maneuvers, (September), 2688-2693. Retrieved from https://scholarsarchive.byu.edu/facpub%0Ahttps://scholarsarchive.byu.edu/facpub/1949
#
# For a reference of the differential flatness proof for the drone dynamics refer
# to: Mellinger, D.,  Kumar, V. (2011). Minimum snap trajectory generation and control for quadrotors. Proceedings - IEEE International Conference on Robotics and 
# Automation, 2520-2525. https://doi.org/10.1109/ICRA.2011.5980409
#
# And this paper for important corrections on the demostration done in the previous paper:
# Faessler, M., Franchi, A., & Scaramuzza, D. (2017). Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories.
# https://doi.org/10.1109/LRA.2017.2776353
#
#					           	
#  The system is x2_dot = A*x_2 + B*u_2
#
#    x2 = [x,y,z,v_x,v_y,v_z,phi,theta,psi].T                            
#
#    u_2 =[u_2a, u_2b].T
#                            
#    A = [[0_3x3, I_3x3, 0_3x3],
#         [0_3x3, 0_3x3, 0_3x3],
#         [0_3x3, 0_3x3, 0_3x3]]
#
#	 B = [[0_3x3, 0_3x3],
#         [I_3x3, 0_3x3], 
#         [0_3x3, I_3x3]]                          
#	 
# The dynamics of this system can be divided as follows, which eases computation and allows
# use of the python control library.    
#                                           __
#  [x_dot ]   = [0 1][ x ]  + [0][u_2a_x]	  |      
#  [v_x_dot]    [0 0][v_x]    [1]   		  |
#											  |
#  [y_dot ]   = [0 1][ y ]  + [0][u_2a_y]     |
#  [v_y_dot]    [0 0][v_y]    [1]			  |---- translation dynamics
#  											  |
#  [z_dot ]   = [0 1][ z ]  + [0][u_2a_z]	  |
#  [v_z_dot]    [0 0][v_z]    [1]			  |
#											__|
#
#                                           __   
#  [phi_dot ]   = [0][phi]  + [0][u_2b_x] 	  |
#											  |
#  [theta_dot ]   = [0][theta]  + [0][u_2b_y] |---- rotational dynamics
#											  |
#  [psi_dot ]   = [0][psi]  + [0][u_2b_z] 	  |
#											__|	
#
# System division that we use to compute constants in a simpler manner.


import control as ctl
print(ctl)
import numpy as np 

# In general    u = Nu*r - K(x -Nx*r) = -K*x + (Nu + K*Nx)*r
# where K are the LQR gains, Nu and Nx are reference input matrices.
# See more at p.493 "Feedback Control of Dynamic Systems, Franklink G,
# Powell, J. Emami-Naeini, Abbas"
# This method calculates Nu and Nx matrices for given A,B,C,D matrices.
def getInputMatrices(A_,B_,C_,D_):

	aug1 = np.concatenate((A_,B_), axis = 1)
	aug2 = np.concatenate((C_,D_), axis = 1)
	# create [[A,B],[C,D]] matrix
	inp = np.concatenate((aug1,aug2), axis = 0)
	#print(inp)

	# create multiplying matrix
	mult = np.zeros((inp.shape[0],1))
	mult[mult.shape[0]-1] = 1.0

	# compute inv(inp)*mult
	inp = np.linalg.inv(inp)*mult

	# nx is all values but last, nu is simply last value
	n_x = inp[0:(inp.shape[0]-1)]
	n_u = inp[-1]

	return n_u, n_x


# Q should be nxn diagonal np.array, where n is dim(A). Diagonal elements should be
# the output performance weights
# R should be mxm np.array where m is dim(B). The diagonal values should be input
# effort weights.
#
# Example:
# output performance matrix for translation variables
#           Qt = np.diag([100.0,1.0])
# input effort matrix for translation variables
#           Rt = np.array([1.0])

def calculate_LQR_gains(A,B,C,D,Q,R):
	# calculate LQR gains
	(K, X, E) = ctl.lqr(A, B, Q, R)

	# calculate Nu, Nx matrices for including reference input
	Nu, Nx = getInputMatrices(A,B,C,D)

	return K, Nu, Nx

# Calculate the gain matrix K required for the poles of a system
#          x_dot = Ax + Bu, with u = -Kx 
# to be equal to the pole array passed in the argument dp.
# dp is an np.array 1xm where m is dim(A)
#
# For the system to be stable, all elements in dp should be < 0
def calculate_pp_gains(A,B,C,D,dp):

	# pole placement via ackermann's formula
	K = ctl.acker(A,B,dp)

	# calculate Nu, Nx matrices for including reference input
	Nu, Nx = getInputMatrices(A,B,C,D)

	return K, Nu, Nx	




# define system matrix for translation variables
# (It is the result of linearization with computer-torque method)
At = np.matrix(
	[[0.0,1.0],
	[0.0,0.0]])

# input matrices
Bt = np.matrix(
	[[0.0],
	[1.0]])

# output matrices
Ct = np.matrix([1.0,0.0]) #np.matrix()

# system matrix for rotational variables
Ar = np.matrix([0.0])

# input matrix
Br = np.matrix([1.0]) 

# output matrix
Cr = np.matrix([1.0])

# transmission matrix for all systems
D_ = np.matrix([0.0])

# PID Gains for 2nd order system  with transfer function
#    H(s) = 1 / s**2
Kpx2 = 3.0
Kix2 = 0.5
Kdx2 = 3.0

Kpy2 = 3.0
Kiy2 = 0.5
Kdy2 = 3.0

Kpz2 = 3.0
Kiz2 = 1.0
Kdz2 = 10.0

# PID Gains for 1st order system with transfer function
#    H(s) = 1 / s
Kp1 = 3
Ki1 = 0.5
Kd1 = 4
	


# Helper code for visualizing the performance of the gains computed 
# by both the LQR and pole placement control methods
if __name__ == '__main__':

	import matplotlib.pyplot as plt

	fig = plt.figure(figsize=(20,10))
	fig.suptitle(" LQR, PP and PID controller performance")
	ax0 = fig.add_subplot(2,1,1)#, projection='3d')
	ax1 = fig.add_subplot(2,1,2)
	#ax2 = fig.add_subplot(3,1,3)#, projection='3d')
	#ax3 = fig.add_subplot(3,2,4)
	#ax4 = fig.add_subplot(3,2,5)
	#ax5 = fig.add_subplot(3,2,6)

	# define simulation time
	t_max = 10
	dt = 0.01
	t = np.arange(0.0,t_max,dt)

	# output performance matrix for translation variables
	Qt = np.diag([100.0,1.0])
	# input effort matrix for translation variables
	Rt = np.array([1.0])	
	# output performance matrix for rotational variables
	Qr = np.diag([5.0])
	# input effort matrix for rotational variables
	Rr = np.array([1.0])

	# Desired pole locations for pole placement method
	# translation and rotatin dynamics poles
	dpt = np.array([-3.0+10j,-3.0-10j])
	dpr = np.array([-8.0])

	Kt_lqr, N_ut_lqr, N_xt_lqr = calculate_LQR_gains(At,Bt,Ct,D_,Qt,Rt)
	Kr_lqr, N_ur_lqr, N_xr_lqr = calculate_LQR_gains(Ar,Br,Cr,D_,Qr,Rr)
	#print(type(Kr_lqr))

	Kt_pp, N_ut_pp, N_xt_pp = calculate_pp_gains(At,Bt,Ct,D_,dpt)
	Kr_pp, N_ur_pp, N_xr_pp = calculate_pp_gains(Ar,Br,Cr,D_,dpr)

	print("Translation dynamics LQR K: {}, Nu: {}, Nx: {}".format(Kt_lqr,N_ut_lqr,N_xt_lqr))
	print("Rotation dynamics LQR K: {}, Nu: {}, Nx: {}".format(Kr_lqr,N_ur_lqr,N_xr_lqr))
	print("Translation dynamics PP K: {}, Nu: {}, Nx: {}".format(Kt_pp, N_ut_pp, N_xt_pp))
	print("Rotation dynamics PP K: {}, Nu: {}, Nx: {}".format(Kr_pp, N_ur_pp, N_xr_pp))

	# define step input gain:  u(t) = R*u(t)
	# for each control variable
	Rx = 10.0
	refx = Rx*np.ones_like(t)
	Rw = 11.0
	refw = Rw*np.ones_like(t)


	# Define 2nd order system transfer function
	num2 = np.array([1.0])
	den2 = np.array([1.0,0.0,0.0])
	plant2 = ctl.tf(num2, den2)

	# Define PID controller for 2nd order system
	num_pid_2 = np.array([Kdz2, Kpz2, Kiz2])
	den_pid_2 = np.array([1.0, 0.0])
	pid2 = ctl.tf(num_pid_2, den_pid_2)

	# Define 1st order system transfer function
	num1 = np.array([1.0])
	den1 = np.array([1.0,0.0])
	plant1 = ctl.tf(num1,den1)

	# Define PID Controller for 1st order system
	num_pid_1 = np.array([Kd1, Kp1])    # np.array([Kd1, Kp1, Ki1])
	den_pid_1 = np.array([1.0])    # np.array([1.0, 0.0])
	pid1 = ctl.tf(num_pid_1, den_pid_1)

	# closed loop dynamics system with LQR
	x_cl_lqr = ctl.ss(At-Bt*Kt_lqr, Bt*(N_ut_lqr + Kt_lqr*N_xt_lqr)*1.0 , Ct, D_)
	w_cl_lqr = ctl.ss(Ar-Br*Kr_lqr, Br*(N_ur_lqr + Kr_lqr*N_xr_lqr)*1.0 , Cr, D_) 

	# closed loop dynamics system with PP
	x_cl_pp = ctl.ss(At-Bt*Kt_pp, Bt*(N_ut_pp + Kt_pp*N_xt_pp)*1.0 , Ct, D_)
	w_cl_pp = ctl.ss(Ar-Br*Kr_pp, Br*(N_ur_pp + Kr_pp*N_xr_pp)*1.0 , Cr, D_) 

	# closed loop dynamics system with PID
	# interconnect plant and pid with feedback block with H(s)= 1
	fb = ctl.tf([1],[1])
	dummy = ctl.series(pid2, plant2)
	pid_controlled_sys2 = ctl.feedback(dummy, fb, sign = -1) 
	fb2 = ctl.tf([1],[1])
	dummy2 = ctl.series(pid1, plant1)
	pid_controlled_sys1 = ctl.feedback(dummy2, fb2, sign = -1)

	# define an input signal shape and draw response
	tx, x_lqr, s = ctl.forced_response(x_cl_lqr, T=t, U=refx)
	tx, w_lqr, s = ctl.forced_response(w_cl_lqr, T=t, U=refw)

	tx, x_pp, s = ctl.forced_response(x_cl_pp, T=t, U=refx)
	tx, w_pp, s = ctl.forced_response(w_cl_pp, T=t, U=refw)

	tx, x_pid, s = ctl.forced_response(pid_controlled_sys2, T=t, U=refx)
	tx, w_pid, s = ctl.forced_response(pid_controlled_sys1, T=t, U=refw)

	#tx, x_ol, s = ctl.forced_response(plant2, T=t, U=refx)

	ax0.plot(t, x_lqr,linestyle = '-',color ='r', label = "x_lqr")
	ax0.plot(t, x_pp,linestyle = '-',color ='g', label = "x_pp")
	ax0.plot(t, x_pid, linestyle = '-',color ='b', label = "x_pid")
	ax0.plot(t, refx,linestyle = '--', color = "k", label = 'x ref')
	ax0.set_title("Step response for translation dynamics", fontsize='small')
	ax0.legend(loc='center right', shadow=True, fontsize='small')
	ax0.set_xlabel("time {s}")
	ax0.set_ylabel("x {m}")

	print("PID Overshoot: {}".format(max(x_pid) - Rx))
	print("")

	ax1.plot(t, w_lqr, linestyle = '-',color ='r', label = "w_lqr")
	ax1.plot(t, w_pp, linestyle = '-',color ='g', label = "w_pp")
	ax1.plot(t, w_pid, linestyle = '-',color ='b', label = "w_pid")
	ax1.plot(t, refw,linestyle = '--', color = "k", label = 'w ref')
	ax1.set_title("Step response of rotational dynamics", fontsize='small')
	ax1.legend(loc='center right', shadow=True, fontsize='small')
	ax1.set_xlabel("time {s}")
	ax1.set_ylabel("w {rad}")

	"""
	ax2.plot(t, x_ol,linestyle = '-',color ='r', label = "x_ol")
	#ax1.plot(t, w_pp,linestyle = '-',color ='g', label = "w_pp")
	ax2.plot(t, refx,linestyle = '--', color = "k", label = 'x ref')
	ax2.set_title("Step response of open loop translation dynamics", fontsize='small')
	ax2.legend(loc='center right', shadow=True, fontsize='small')
	ax2.set_xlabel("time {s}")
	ax2.set_ylabel("x {m}")
	"""
	plt.show()





