cmake_minimum_required(VERSION 3.0.2)
project(mission)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  ct_core
  rospack
  roslib
  genmsg
  actionlib_msgs
  actionlib
  message_generation
  slam
  dynamic_reconfigure
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Mission.cfg
)

#add_action_files(DIRECTORY action FILES Navigation.action)

add_message_files(
  FILES
  RotationalMovement.msg
  SinTraversalPath.msg
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
#  CATKIN_DEPENDS actionlib_msgs
#	INCLUDE_DIRS include
 CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs message_runtime slam
#	DEPENDS system_lib
)

include_directories(
	# include
  ${catkin_INCLUDE_DIRS}
)

#catkin_install_python(PROGRAMS
#  scripts/go_to_target.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
