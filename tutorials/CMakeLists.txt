cmake_minimum_required(VERSION 3.0.2)
project(tutorials)


find_package(catkin REQUIRED COMPONENTS
  controls
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorials
#  CATKIN_DEPENDS controls nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
