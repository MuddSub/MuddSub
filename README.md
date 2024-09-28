# MuddSub
The code for MuddSub's Alfie AUV

## Installation

### Prerequisites
This all needs to be in Ubuntu, preferably 18.04. Other linux distributions can in theory be used, but not all packages are guaranteed to be supported. For example, in Arch linux, everything except simulation is supported. These instructions are all written for Ubuntu. If you want to use a different distribution, you'll have to figure out how to install the packages in the distro.

Start by following [these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ros melodic.

You'll also need to install a few dependencies to get everything working:

```
sudo apt-get install ros-melodic-uwsim
sudo apt-get install python2.7-dev
sudo apt-get install python-catkin-tools
sudo apt-get install ros-melodic-tf
sudo apt-get install ros-melodic-urdf
sudo apt-get install libeigen3-dev
sudo apt-get install ros-melodic-tf2-geometry-msgs
sudo apt-get install ros-melodic-vision-msgs
```

The first is optional, and provides the tools we use for simulation. The second is a dependency for a library we use for controls. The third gives an alternate build tool to the one which comes standard with ROS. We build with `catkin build` instead of `catkin_make` because it allows standard CMake projects to be compiled within our workspace. The last two are used to generate documentation.

### Download and Build



You'll need a catkin workspace, which can be created by opening a terminal and running
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin build
```

Enter the directory and clone this repository:

```
cd ~/catkin_ws/src
git clone https://github.com/MuddSub/MuddSub.git --recurse-submodules -j8
```

With that said, this is a good opportunity to build!

```
catkin build muddsub_sim
```
This may take a few minutes! Note: If you didn't install `ros-melodic-uwsim`, you can still build, but with `catkin build muddsub_primary` instead. This builds everything but the simulator.

### Overview of system

Important files:
- drivers/src/teensy/fullTeensy/TeensyCode.ino:
    - The teensy sends PWMs to the electronic speed controllers (ESCs).
    - It reads the PWMs from a serial port and prints depth sensor values (in meters) to the same serial port.
    - Reads from mission on/off switch which when on starts our state machine.

- drivers/scripts/teensy_interface.py
    - Opens Serial Connection to teensy.
    - Writes thruster pwm values from ros topics robot/pwm/... and formats as string to send to Teensy.
    - Publishes the on/off switch's state to robot/mission_started.

- mission/scripts/Mission.py
    - Contains the code that actually controls the robot
    - Publishes to the pwm topics
