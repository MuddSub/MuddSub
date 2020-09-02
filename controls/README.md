# Controls


The controls package implements vehicle dynamics and the control systems for our AUV.

Maintainer: [Seth Isaacson](sisaacson@hmc.edu).

Status: Mostly working, but not well tested! This is a student project which is still in progress, so there may be some issues. The good news is we're all happy to help you fix them, as that will make it better for everyone!

## Vehicle Dynamics

This robot operates in 12 degrees of freedom. Those are defined as follows:

The linear location of the robot is the x,y, and z positions. We use the north-east-down (NED) frame, which is common in underwater robotics.
this means that x points forward, y points right, and z points down. These are commonly referred to as surge, sway, and heave respectively.

The angular location of the robot is the rotation about the x,y, and z axes. The x axis rotation is called "roll", which can be remembered
because that's the axis you would spin around to make a plane do a barrel roll. The y axis rotation is called pitch (you pitch forwards and backwards).
Finally, the most useful rotation is about the z axis, and called "yaw". 

The other 6 degrees of freedom are the velocities about all of those axes.

![axes](https://www.mdpi.com/sensors/sensors-15-07016/article_deploy/html/images/sensors-15-07016-g001.png)

The vehicle dynamics are taken from the classic text in marine vehicles, which is Thor Fossen's Handbook of Marine Craft Hydrodynamics and Motion Control (2011). 
The dynamics are implemented using the wonderful c++ [control toolbox](https://github.com/ethz-adrl/control-toolbox). This allows for easy linearization of the dynamics,
as well as solving optimal control problems.

## Control Systems

Alfie uses three separate controllers. The first two are simple PID controllers which try to keep the robot flat in the water. So, one controller tries to keep pitch
at 0, and the other tries to keep roll at 0. The third controller is an 8DOF LQR (linear quadratic regulator) which controls x,y,z,yaw, and those velocities.

For more details on PID, see: https://en.wikipedia.org/wiki/PID_controller

For details on LQR, see: https://www.mathworks.com/videos/state-space-part-4-what-is-lqr-control-1551955957637.html

Or, just ask Seth! I'm happy to explain more about controls and how you can contribute.

## Simulator

Our simulator is based on [uwsim](http://www.irs.uji.es/uwsim/). To implement physics, we use our own model of vehicle dynamics. This needs work to be more realistic and
incorporate more sensors, but it's a good start for modeling vehicle dynamics and control systems.



### Running the simultator:

You'll need UWSim to run the simulator. This is only tested on ROS melodic. Unfortunately UWSim is quite hard to install from source, and the only package manager
I've found which installs it correctly is `apt`. So, if you're not on Ubuntu (or maybe some other Debian-esque linux), you may have a hard time. 

Start the visual simulator:
```
cd ~/catkin_ws/src/MuddSub/
roscore
rosrun uwsim uwsim 
```

Launch vehicle dynamics (in another terminal):
```
roslaunch controls DynamicSimulation.launch
```

Then, you can set the setpoint by publishing to the `/robot_setpoint` topic.

Currently, this is not a very streamlined experience, and you're unfortunately likely to run into bugs getting the simulator running. UWSim is great, but very picky,
and this has only been tested on two machines. Please do not hesitate to email Seth with any questions, and I'm happy to help you debug! 

## Known Limitations
- Currently, the tuning abilility of the control systems is very limited, as it must be hardcoded. This will be a good project for a new member in the fall.
- Since thus far this has only been used in simulation, we need a thruster allocation system. This should be easy to reconfigure in the case of motor failure. Yet another good project for new members!

## Future Work

- The major next steps is to switch from LQR control to linear model predictive control (MPC). While not strictly neccesary, it will be a fun project, and may be useful.
- We also need better tuning and visualizing tools.
- The simulator needs noise injection to be more realistic. We also need to add more of our sensors to the simulation. This will be more good projects for new members.
- The simulator needs the robosub course modeled in using URDF (universal robot description format). This is, you guessed it, a great project for new members.
