import numpy as np
import matplotlib.pyplot as plt 

import control.dif_flat as df_flat 
import control.dif_flat_rotor_drag as df_flat_drag
import utils.utils as utils
import model.params as params


def gen_helix_trajectory2(t):
    """
        This function returns the trajectory: position, velocity,
        acceleration, jerk and snap an object going through a 3D helix 
        should have.
    """
    a = 2.0
    b = 2.0
    c = 5.0

    wx = 0.5
    wy = 1.0

    x_0 = 1.0
    y_0 = 1.0
    z_0 = 0.0

    # positions in helix
    x = a*np.cos(wx*t) + x_0
    y = b*np.sin(wy*t) + y_0
    z = c*t
    #psi = 0.0*np.ones_like(t)
    #tangent_vector = map(lambda a,b,c: np.matrix([[a],[b],[0]]),-a*wx*np.sin(wx*t),b*wy*np.cos(wy*t),c)
    psi = np.sin(t)
    #psi = np.arccos( )

    # velocities in helix
    v_x = -a*wx*np.sin(wx*t)
    v_y = b*wy*np.cos(wy*t)
    v_z = c*np.ones_like(t)
    psi_rate = np.cos(t)#0.0*np.ones_like(t)

    # accelerations in helix
    a_x = -(wx**2)*(x - x_0)
    a_y = -(wy**2)*(y - y_0)
    a_z = 0.0*np.ones_like(t)
    psi_dd = -1.0*np.sin(t)#0.0*np.ones_like(t)

    # jerks in helix
    j_x = -(wx**2)*(v_x)
    j_y = -(wy**2)*(v_y)
    j_z = 0.0*np.ones_like(t)
    psi_ddd = -1.0*np.cos(t)#0.0*np.ones_like(t)

    # snap in helix
    s_x = -(wx**2)*(a_x)
    s_y = -(wy**2)*(a_y)
    s_z = 0.0*np.ones_like(t)
    psi_dddd = np.sin(t) #0.0*np.ones_like(t)

    # pack everything
    pos = np.array([[x],[y],[z]])
    vel = np.array([[v_x],[v_y],[v_z]])
    acc = np.array([[a_x],[a_y],[a_z]])
    jerk = np.array([[j_x],[j_y],[j_z]])
    snap = np.array([[s_x],[s_y],[s_z]])

    return [pos,vel,acc,jerk,snap, psi, psi_rate, psi_dd, psi_ddd, psi_dddd]


def draw_output(states, figs):


    # Euler angles
    phi1 = map(lambda a: a[2][0][0]*180.0/np.pi, states)
    theta1 = map(lambda a: a[2][1][0]*180.0/np.pi, states)
    psi1 = map(lambda a: a[2][2][0]*180.0/np.pi, states)

    figs[0] = utils.add_plots(figs[0],sim_time,[phi1,theta1,psi1],["-","-","-"],["r","g","b"],["phi","theta","psi"],"Angular position of quadrotor",'t {s}','phi, theta, psi {degree}')
    figs[0].legend(loc='lower right', shadow=True, fontsize='small')    

    # Angular velocities
    wx1 = map(lambda a: a[3][0][0], states)
    wy1 = map(lambda a: a[3][1][0], states)
    wz1 = map(lambda a: a[3][2][0], states)
    #print(wx1)
    figs[1] = utils.add_plots(figs[1],sim_time,[wx1,wy1,wz1],["-","-","-"],["r","g","b"],["wx","wy","wz"],"Angular velocities of quadrotor",'t {s}','wx, wy, wz {degree/s}')
    figs[1].legend(loc='lower right', shadow=True, fontsize='small')

    # Angular accelerations
    wx1_dot = map(lambda a: a[4][0][0], states)
    wy1_dot = map(lambda a: a[4][1][0], states)
    wz1_dot = map(lambda a: a[4][2][0], states)
    #print(wx1)
    figs[2] = utils.add_plots(figs[2],sim_time,[wx1_dot,wy1_dot,wz1_dot],["-","-","-"],["r","g","b"],["wx_dot","wy_dot","wz_dot"],"Angular acceleration of quadrotor",'t {s}','wx, wy, wz {degree/s}')
    figs[2].legend(loc='lower right', shadow=True, fontsize='small')    

    # control torque
    ux1 = map(lambda a: a[9][0][0], states)
    uy1 = map(lambda a: a[9][1][0], states)
    uz1 = map(lambda a: a[9][2][0], states)
    #print(wx1)
    figs[3] = utils.add_plots(figs[3],sim_time,[ux1,uy1,uz1],["-","-","-"],["r","g","b"],["ux","uy","uz"],"Angular acceleration of quadrotor",'t {s}','wx, wy, wz {degree/s}')
    figs[3].legend(loc='lower right', shadow=True, fontsize='small')    

    # control thrust
    F_t = map(lambda a: a[8], states)
    weight = params.mass*params.g*np.ones_like(F_t)
    figs[4] = utils.add_plots(figs[4],sim_time,[F_t,weight],["-","--"],["r","k"],["F","m*g"],"Rotor Thrust -F- over time",'t {s}','F {N}')
    figs[4].legend(loc='lower right', shadow=True, fontsize='small') 


if __name__ == '__main__':

    tmax = 10
    step = 0.1
    sim_time = np.arange(0, tmax, step)
    
    ref_states1 = list()
    ref_states2 = list()
    
    for t in sim_time:
        flat_out_traj = gen_helix_trajectory2(t)
        ref_states1.append(df_flat.compute_ref(flat_out_traj))
        ref_states2.append(df_flat_drag.compute_ref(flat_out_traj))

    # convert to np.array
    ref_states1 = map(lambda a: map(lambda b: np.array(b),a), ref_states1)


    fig0 = plt.figure(figsize=(20,10))
    fig0.tight_layout()
    fig0ax0 = fig0.add_subplot(3,2,1)
    fig0ax1 = fig0.add_subplot(3,2,2)
    fig0ax2 = fig0.add_subplot(3,2,3)
    fig0ax3 = fig0.add_subplot(3,2,4)
    fig0ax4 = fig0.add_subplot(3,2,5)
    fig0ax5 = fig0.add_subplot(3,2,6) 
    fig0.suptitle("Differential Flatness")  

    fig1 = plt.figure(figsize=(20,10))
    fig1.tight_layout()
    fig1ax0 = fig1.add_subplot(3,2,1)
    fig1ax1 = fig1.add_subplot(3,2,2)
    fig1ax2 = fig1.add_subplot(3,2,3)
    fig1ax3 = fig1.add_subplot(3,2,4)
    fig1ax4 = fig1.add_subplot(3,2,5)
    fig1ax5 = fig1.add_subplot(3,2,6)
    fig1.suptitle("Differential Flatness with Drag") 

    figlist0 = [fig0ax0, fig0ax1, fig0ax2, fig0ax3, fig0ax4, fig0ax5]
    draw_output(ref_states1, figlist0)
    figlist1 = [fig1ax0, fig1ax1, fig1ax2, fig1ax3, fig1ax4, fig1ax5]
    draw_output(ref_states2, figlist1)



    plt.show()