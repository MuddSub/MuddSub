# MuddSub
The code for MuddSub's Alfie AUV

## Installation

### Prerequisites
This all needs to be in Ubuntu, preferably 20.04. Other linux distributions can in theory be used, but not all packages are guaranteed to be supported. For example, in Arch linux, everything except simulation is supported. These instructions are all written for Ubuntu. If you want to use a different distribution, you'll have to figure out how to install the packages in the distro.

Start by following [these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ros melodic.

You'll also need to install a few dependencies to get everything working:

```
sudo apt-get install python2.7-dev
sudo apt-get install python-catkin-tools
sudo apt-get install ros-noetic-tf
sudo apt-get install ros-noetic-urdf
sudo apt-get install libeigen3-dev
sudo apt-get install ros-noetic-tf2-geometry-msgs
sudo apt-get install ros-noetic-vision-msgs
sudo apt-get install doxygen
pip install doxypypy
```

The first is a dependency for a library we use for controls. The second gives an alternate build tool to the one which comes standard with ROS. We build with `catkin build` instead of `catkin_make` because it allows standard CMake projects to be compiled within our workspace. The last two are used to generate documentation.

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
This may take a few minutes! 
