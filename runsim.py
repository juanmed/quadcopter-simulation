"""
author: Peter Huang, Antonio Cuni
email: hbd730@gmail.com, anto.cuni@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

from quadPlot import plot_quad_3d

from control import lqr_controller as lqr
from control import pid_controller as pid

import trajGen
import trajGen3D
import model.params as params
from model.quadcopter import Quadcopter
import numpy as np

import matplotlib.pyplot as plt

animation_frequency = 50
control_frequency = 200 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]


# variables to plot
F_t = list()  # Thrust
M_t = list()  # Torque
t_s = list()  # simulation time
d_s = list()  # desired states
q_s = list()  # quadrotor states


def record(name):
    fig0 = plt.figure(figsize=(20,10))
    fig0.tight_layout()
    fig0ax0 = fig0.add_subplot(3,2,1)
    fig0ax1 = fig0.add_subplot(3,2,2)
    fig0ax2 = fig0.add_subplot(3,2,3)
    fig0ax3 = fig0.add_subplot(3,2,4)
    fig0ax4 = fig0.add_subplot(3,2,5)
    fig0ax5 = fig0.add_subplot(3,2,6)

    fig1 = plt.figure(figsize=(20,10))
    fig1.tight_layout()
    fig1ax0 = fig1.add_subplot(3,2,1)
    fig1ax1 = fig1.add_subplot(3,2,2)
    fig1ax2 = fig1.add_subplot(3,2,3)
    fig1ax3 = fig1.add_subplot(3,2,4)
    fig1ax4 = fig1.add_subplot(3,2,5)
    fig1ax5 = fig1.add_subplot(3,2,6)

    # Thrust
    fig0ax0.plot(t_s, F_t, linestyle = "-", color = "r", label = "F")
    fig0ax0.plot(t_s, params.mass*params.g*np.ones_like(t_s), linestyle="--", color = "k", label = "m*g")
    fig0ax0.set_title("Rotor Thrust -F- over time")
    fig0ax0.set_xlabel('t {s}')
    fig0ax0.set_ylabel('F {N}')
    fig0ax0.legend(loc='lower right', shadow=True, fontsize='small') 

    # Torques
    u2 = map(lambda a: a[0],M_t) # extract u2 for all points in time
    u3 = map(lambda a: a[1],M_t)
    u4 = map(lambda a: a[2],M_t)

    fig0ax1.plot(t_s, u2,  linestyle = "-", color = "r", label = "u2")
    fig0ax1.plot(t_s, u3,  linestyle = "-", color = "g", label = "u3")
    fig0ax1.plot(t_s, u4,  linestyle = "-", color = "b", label = "u4")
    fig0ax1.set_title("Components of torque vector M over time")
    fig0ax1.set_xlabel('t {s}')
    fig0ax1.set_ylabel('u2,u3,u4 {N*m}')
    fig0ax1.legend(loc='lower right', shadow=True, fontsize='small')

    # X position
    q_x = map(lambda a: a[0][0], q_s)   # get quad x position
    d_x = map(lambda a: a.pos[0], d_s)  # get desired x position
    x_e = map(lambda a,b: a-b,d_x,q_x)  # compute error

    fig0ax2.plot(t_s, q_x, linestyle = "-", color = "g", label = "quad x")
    fig0ax2.plot(t_s, d_x, linestyle = "--", color = "r", label = "des x")
    fig0ax2.plot(t_s, x_e, linestyle = "-", color = "b", label = "x error")
    fig0ax2.set_title("X - axis position of quadrotor")
    fig0ax2.set_xlabel('t {s}')
    fig0ax2.set_ylabel('x {m}')
    fig0ax2.legend(loc='lower right', shadow=True, fontsize='small')

    # Y position
    q_y = map(lambda a: a[0][1], q_s)
    d_y = map(lambda a: a.pos[1], d_s)
    y_e = map(lambda a,b: a-b,d_y,q_y)

    fig0ax3.plot(t_s, q_y, linestyle = "-", color = "g", label = "quad y")
    fig0ax3.plot(t_s, d_y, linestyle = "--", color = "r", label = "des y")
    fig0ax3.plot(t_s, y_e, linestyle = "-", color = "b", label = "y error")
    fig0ax3.set_title("Y - axis position of quadrotor")
    fig0ax3.set_xlabel('t {s}')
    fig0ax3.set_ylabel('y {m}')
    fig0ax3.legend(loc='lower right', shadow=True, fontsize='small')

    # Z position
    q_z = map(lambda a: a[0][2], q_s)
    d_z = map(lambda a: a.pos[2], d_s)
    z_e = map(lambda a,b: a-b,d_z,q_z)

    fig0ax4.plot(t_s, q_z, linestyle = "-", color = "g", label = "quad z")
    fig0ax4.plot(t_s, d_z, linestyle = "--", color = "r", label = "des z")
    fig0ax4.plot(t_s, z_e, linestyle = "-", color = "b", label = "z error")
    fig0ax4.set_title("Z - axis position of quadrotor")
    fig0ax4.set_xlabel('t {s}')
    fig0ax4.set_ylabel('y {m}')
    fig0ax4.legend(loc='lower right', shadow=True, fontsize='small')

    # Euler angles
    q_phi = map(lambda a: a[2][0]*180.0/np.pi, q_s)
    q_theta = map(lambda a: a[2][1]*180.0/np.pi, q_s)
    q_psi = map(lambda a: a[2][2]*180.0/np.pi, q_s)

    fig0ax5.plot(t_s, q_phi, linestyle = "-", color = "r", label = "phi")
    fig0ax5.plot(t_s, q_theta, linestyle = "-", color = "g", label = "theta")
    fig0ax5.plot(t_s, q_psi, linestyle = "-", color = "b", label = "psi")
    fig0ax5.set_title("Angular position of quadrotor")
    fig0ax5.set_xlabel('t {s}')
    fig0ax5.set_ylabel('phi, theta, psi {degree}')
    fig0ax5.legend(loc='lower right', shadow=True, fontsize='small')

    #  X Linear velocity
    q_vx = map(lambda a: a[1][0], q_s)
    d_vx = map(lambda a: a.vel[0], d_s)   
    vx_e = map(lambda a,b: a-b,d_vx,q_vx)

    fig1ax0.plot(t_s, q_vx, linestyle = "-", color = "g", label = "quad Vx")
    fig1ax0.plot(t_s, d_vx, linestyle = "--", color = "r", label = "des Vx")
    fig1ax0.plot(t_s, vx_e, linestyle = "-", color = "b", label = "Vx error")
    fig1ax0.set_title("X axis linear Velocities of quadrotor")
    fig1ax0.set_xlabel('t {s}')
    fig1ax0.set_ylabel('Vx {m/s}')
    fig1ax0.legend(loc='lower right', shadow=True, fontsize='small')   

    #  Y Linear velocity
    q_vy = map(lambda a: a[1][1], q_s)
    d_vy = map(lambda a: a.vel[1], d_s)   
    vy_e = map(lambda a,b: a-b,d_vy,q_vy)

    fig1ax1.plot(t_s, q_vy, linestyle = "-", color = "g", label = "quad Vy")
    fig1ax1.plot(t_s, d_vy, linestyle = "--", color = "r", label = "des Vy")
    fig1ax1.plot(t_s, vy_e, linestyle = "-", color = "b", label = "Vy error")
    fig1ax1.set_title("Y axis linear Velocities of quadrotor")
    fig1ax1.set_xlabel('t {s}')
    fig1ax1.set_ylabel('Vy {m/s}')
    fig1ax1.legend(loc='lower right', shadow=True, fontsize='small')  

    #  Z Linear velocity
    q_vz = map(lambda a: a[1][2], q_s)
    d_vz = map(lambda a: a.vel[2], d_s)   
    vz_e = map(lambda a,b: a-b,d_vz,q_vz)

    fig1ax2.plot(t_s, q_vz, linestyle = "-", color = "g", label = "quad Vz")
    fig1ax2.plot(t_s, d_vz, linestyle = "--", color = "r", label = "des Vz")
    fig1ax2.plot(t_s, vz_e, linestyle = "-", color = "b", label = "Vz error")
    fig1ax2.set_title("Z axis linear Velocities of quadrotor")
    fig1ax2.set_xlabel('t {s}')
    fig1ax2.set_ylabel('Vz {m/s}')
    fig1ax2.legend(loc='lower right', shadow=True, fontsize='small')  

    # Angular velocities

    # Euler angles
    q_wx = map(lambda a: a[3][0]*180.0/np.pi, q_s)
    q_wy = map(lambda a: a[3][1]*180.0/np.pi, q_s)
    q_wz = map(lambda a: a[3][2]*180.0/np.pi, q_s)

    fig1ax3.plot(t_s, q_wx, linestyle = "-", color = "r", label = "wx")
    fig1ax3.plot(t_s, q_wy, linestyle = "-", color = "g", label = "wy")
    fig1ax3.plot(t_s, q_wz, linestyle = "-", color = "b", label = "wz")
    fig1ax3.set_title("Angular velocities of quadrotor")
    fig1ax3.set_xlabel('t {s}')
    fig1ax3.set_ylabel('wx, wy, wz {degree/s}')
    fig1ax3.legend(loc='lower right', shadow=True, fontsize='small')

    # save
    fig0.savefig("t_"+name, dpi = 200) #translation variables
    fig1.savefig("r_"+name, dpi = 200) #rotation variables

def attitudeControl(quad, time, waypoints, coeff_x, coeff_y, coeff_z):
    desired_state = trajGen3D.generate_trajectory(time[0], 1.2, waypoints, coeff_x, coeff_y, coeff_z)
    F, M = pid.run(quad, desired_state)
    quad.update(dt, F, M)
    time[0] += dt

    # save variables to graph later
    F_t.append(F)
    M_t.append(M)
    t_s.append(time[0])
    d_s.append(desired_state)
    q_s.append([quad.state[0:3],quad.state[3:6],quad.attitude(),quad.state[10:13]])

def main():
    pos = (0.5,0,0)
    attitude = (0,0,0)
    quadcopter = Quadcopter(pos, attitude)
    waypoints = trajGen3D.get_helix_waypoints(6, 9)
    (coeff_x, coeff_y, coeff_z) = trajGen3D.get_MST_coefficients(waypoints)

    def control_loop(i):
        for _ in range(control_iterations):
            attitudeControl(quadcopter, time, waypoints, coeff_x, coeff_y, coeff_z)
        return quadcopter.world_frame()

    plot_quad_3d(waypoints, control_loop)

    if(True): # save inputs and states graphs
        print("Saving figures...")
        record("pid.jpg")
    print("Closing.")

if __name__ == "__main__":
    main()
