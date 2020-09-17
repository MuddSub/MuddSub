# Tutorial 1: ROS and Vehicle Dynamics

Welcome to MuddSub! This tutorial will introduce you to Robot Operating System (ROS) and implementing a simple PID (proportional, integral, derivative) controller to control the depth of a simulated robot.


## Part 1: Setup
Before you get started, you need to download and build our code. To do that, go to the root repository at [github.com/muddsub/muddsub](github.com/muddsub/muddsub) and follow the
instructions to download and build the code. When you're done with that, come back here.

Navigate to your workspace (probably named `catkin_ws`) with `cd ~/catkin_ws`. Then, run `source devel/setup.bash`. This configures all of the ROS tools in your current terminal.

Then, open two terminals (or, one Terminator window with two panes). In one of the terminals, run `roscore`. This starts the ros backend. In the other terminal,
run `rosrun uwsim uwsim`. If you get a note saying that `uwsim` is not a package, run `source devel/setup.bash` if you haven't already. Otherwise, make
sure you installed all of the recommended packages listed on the main README. Specifically, run `sudo apt-get install ros-melodic-uwsim` again.

`uwsim` will ask you for permission to download some data. Say yes. Then, you should (eventually) see a window pop up with a visual simulator.

## Part 2: Build our simulation files

There are two commands which you need to run to configure our simulator to load the model of our robot. Run these:

```
cd ~/catkin_ws/src/MuddSub
rosrun xacro xacro core/descriptions/alfie.urdf.xacro -o core/descriptions/alfie.urdf
rosrun xacro xacro sim/scenes/GateScene.xml.xacro -o sim/scenes/GateScene.xml
```

Now, you can run the simulator with our robot! This will also allow us to command the position of the robot using ROS messages.

```
roslaunch sim sim.launch
```

## Part 3: Implementing a PID Controller

For some backgroun on PID controllers, check out [this](https://www.youtube.com/watch?v=wkfEZmsQqiA) video, or ping any of the engineers on the team!

In this lab, we're trying to control the depth of the robot using a PID controller. So, the __setpoint__ of the robot is the desired depth, and the __plant state__
is the current depth. Therefore, the __error__ is defined by

`error = plantState - setpoint`

We can find the derivative of that as

`errorDerivative = (error - previousError)/deltaT`

where deltaT is the time passed since the last time step.

Likewise, we can integrate it as:

`errorIntegral = errorIntegral + error*deltaT`

Then, the control effort is computed as follows:

`controlEffort = error*kP + errorDeriv*kD + errorIntegral*kI`

Where `kP, kI`, and `kD` are user-set __control gains__.

### Implementing the controller

### Computing the Errors

Open the `ControllerTemplate.py` file in this directory. When you're done working on this section, rename this file to `Controller.py`, and rename the existing `Controller.py` to something else.

In the function `updateErrors()`, you'll compute `error`, `errorDerivative`, and `errorIntegral`. The results should be stored in the corresponding
members of the Controller class.

You can access the `plantState` with `self.model.getZPosition()`, and the `setpoint` with `self.setpoint`. Compute the three errors (normal, derivative, and integral)
as described, and store the results in `self.error`, `self.derivativeError`, and `self.integralError`, respectively.


#### Fixing the derivative term.
We'll run into a small problem with our derivative term. Basically, if the state of the robot changes very quickly, our computed `errorDerivative` will spike.
This can cause issues because we may see a near-infinite derivative, which will ruin the effectiveness of our system. So, we must **filter** the output. We'll use
a simple filter called an __infinite impule responese__ filter. Basically, we will do the following:

`errorDerivative = 0.7*previousErrorDerivative + 0.3 * errorDerivative`

This forces the derivative term to change much slower, and works surprisingly well! Modify the `updateErrors()` function to compute this filtered error as described above.
If you get stuck, you can always reference the solution file (provided in this directory!).

#### Computing the control

Now that we have computed the errors, we can use this to compute the __control effort__, which basically describes how hard we "push" to get from the current
location to the setpoint.

There are three variables in the class, `kP`, `kI`, and `kD` which are useful for this. The control output is simply computed as
`controlEffort = kP*error + kI * errorIntegral + kD * errorDerivative`

Update the `computeControl` function to return this value.

### Implementing the subscribers

ROS Subscribers will be used to fetch values for `kP`, `kI`, `kD`, and `setpoint`. The first of these subscribers is already implemented in `ControllerTemplate.py` for you.

A subscriber has two parts: a callback, and the subscriber itself. The callback specifies what to do with a recieved message. The subscriber tells ROS to call the callback every time
a message is received.

For example, in the template file, `kpCallback(msg)` takes a message of type Float64, which has a single member, named data. See [this link](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html) for the official documentation.
Then, this line of code registers the subscriber, and tells it to call kpCallback every time a message is recieved on channel `/kP`. The message is automatically passed to kpCallback
as the argument.

`kPSub = rospy.Subscriber("kP", Float64, kpCallack)`

Implement the other three callbacks subscribers, as described in the template.


### Running it

Rename the existing `Controller.py` to something else (maybe `ControllerSolution.py`). Then, rename your file from `ControllerTemplate.py` to `Controller.py`.

Now, we need to make it executable. You can do this with `chmod u+x ~/catkin_ws/src/muddsub/tutorials/dynamics/Controller.py`

Finally, you can run everything together with `roslaunch tutorials DynamicsSim.launch`. This will run the simulator, and your code.

## Part 4: Tuning the PID Loops.

Now, on to the fun part! We want to 'tune' the PID loop so it performs as well as possible. A well-tuned loop will go to the exact position you specify,
quickly, and without any unwanted oscillations. To tune the loops, we modify the gains, `kP`, `kI`, and `kD`. For our purposes, `kI` will always be zero.
Note that this makes our PID controller into PD controller, since there is no longer an Ingegral term.

To send the robot to depth, open up a terminal, and run `rostopic pub /setpoint std_msgs/Float64 "data: 1.0"`

You should see the robot go to depth, but oscillate a bit. Try to play around with the proportional and derivative terms to change the behavior. For example, 
what happens when you increase `kP` or `kD`.

To change `kP`, you can use:

`rostopic pub /kP std_msgs/Float64 "data: <whatever value you want>"`

`kD` is the same, but replace `/kP` with `/kD`. Note that this will trigger the subscribers that you declared in the controller! I've provided you with approximate
gains to start with of `kP = 0.00001` and `kD = 0.005`. Note that making either of these values too big will cause the robot to wildly thrash around, and it
may even appear to dissapear because it's moving too fast. In this case, quit the simulation (by entering control+c in the terminal you ran the `roslaunch` command from),
and restart.

This process may take some time as you build intuition for what the different terms do. Please reach out with questions, and happy tuning!

It may be helpful to see some information about the robot while it's running. You can add this line to your `computeControl()` function to
print the error, as well as the proportional and derivative controls:

`rospy.loginfo("Error: {}, P: {}, D: {}".format(self.error, P, D))`
