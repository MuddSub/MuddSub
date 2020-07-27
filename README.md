# MuddSub
The code for MuddSub's Alfie AUV

## Installation

This all needs to be in Ubuntu, preferably 18.04. Other linux distributions can in theory be used, but not all packages are guaranteed to be supported. For example, in Arch linux, everything except simulation is supported. These instructions are all written for Ubuntu. If you want to use a different distribution, you'll have to figure out how to install the packages in the distro.

Start by following [these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ros melodic. 

You'll need a catkin workspace, which can be created by opening a terminal and running `mkdir -p ~/catkin_ws/src`.

Enter the directory and clone this repository:

```
cd ~/catkin_ws/src 
git clone https://github.com/MuddSub/MuddSub.git --recurse-submodules -j8
```

You'll need to install a few dependencies to get everything working:

```
sudo apt-get install ros-melodic-uwsim
sudo apt-get install python2.7-dev
sudo apt-get install python-catkin-tools
```

The first is optional, and provides the tools we use for simulation. The second is a dependency for a library we use for controls. The third gives an alternate build tool to the one which comes standard with ROS. We build with `catkin build` instead of `catkin_make` because it allows standard CMake projects to be compiled within our workspace.

With that said, this is a good opportunity to build!

```
cd .. 
catkin build
```

