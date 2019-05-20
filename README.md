Quadcopter 3D Simulator
-----

The project simulates a quadcopter in 3D environment. It contains a basic quadcopter dynamics model, hover controller, trajectory generator, visualisation toolkit and a top level scheduler which runs each module at a specific rate.

#### PID Controller

![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/pid.gif)
![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/t_pid.jpg)

#### LQR Controller

![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/lqr.gif)
![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/t_lqr.jpg)

#### Differential Flatness based controller (euler angle orientation representation)
![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/df.gif)
![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/t_df.jpg)

## Quarotor model considering rotor drag

Rotor drag is modeled as in reference [1] for rigid propellers. Adding this important effect results in a more realistic simulation of the drone dynamics. Also, the performance of control algorithms considering aerodynamic effects can be assesed.

#### Differential Flatness based controller with no rotor drag aerodynamics consideration

![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/df1_airdrag.gif)
![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/t_df1_airdrag.jpg)


#### Differential Flatness based controller with velocity-only dependent rotor drag [2]

![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/df2_airdrag.gif)
![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/t_df2_airdrag.jpg)

#### Differential Flatness based controller with velocity-and-thrust dependent rotor drag

![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/df2_airdrag.gif)
![alt tag](https://github.com/juanmed/quadcopter-simulation/blob/master/t_df2_airdrag.jpg)

Motivation
-----
I have been playing and studying quadcopter in my spare time since 2014 when I first bought a crazyflie. There are many interesting projects around already, like vision-based SLAM, hover control and advance manoeuvre, etc. However, there are very few open source quadcopter simulator which helps a beginner to overcome the mental barrier of understanding the underlying physics. There are a lot of research papers on the topic of quadcopter control and autonomous application, but none of those can be made possible without a decent simulation tool. This project aims to address that. Thanks to Coursera's online course Aerial Robotics by Professor Vijay Kumar, which presents quadcopter's motion equations in detail, I was then inspired and finally able to write this from scratch.

Install
-----
The simulator uses numpy, matplotlib, control, slycot and scipy.

1. Install [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git).
2. In the location you prefer, open a terminal and:
```bash
git clone https://github.com/juanmed/quadcopter-simulation
python -m pip2 install -r requirements.txt
```
To Run
-----

```bash 
python runsim.py
```

Future Work
-----
1. add sensor noise model, wind.
2. add sensor fusion.i.e EKF,UKF
3. reinforment learning.

Contribution
-----
Any contribution are welcome if you are interested in the project, but please let me know.

References
-----

[1] Kai, J. M., Allibert, G., Hua, M. D., & Hamel, T. (2017). Nonlinear feedback control of Quadrotors exploiting First-Order Drag Effects. IFAC-PapersOnLine, 50(1), 8189-8195. https://doi.org/10.1016/j.ifacol.2017.08.1267
[2] Faessler, M., Franchi, A., & Scaramuzza, D. (2017). Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories. https://doi.org/10.1109/LRA.2017.2776353

Contact
-----
Peter Huang

Email: hbd730@gmail.com
