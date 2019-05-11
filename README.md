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

Contact
-----
Peter Huang

Email: hbd730@gmail.com
