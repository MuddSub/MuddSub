cmake_minimum_required(VERSION 3.0.2)
project(motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
	actionlib
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system)


# Generate actions in the 'action' folder
add_action_files(
	DIRECTORY action
  FILES
  CreatePath.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs#   std_msgs
	actionlib_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_planning
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(MotionPlanningExampleNode
	src/MotionPlanningExampleNode.cc
  src/MotionPlanningPublisher.cc
)
add_dependencies(MotionPlanningExampleNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MotionPlanningExampleNode ${catkin_LIBRARIES})

add_executable(path_server
	src/PathServer.cc
  src/PathServerNode.cc
)
target_link_libraries(
	path_server
  ${catkin_LIBRARIES}
)
add_dependencies(
  path_server
  ${motion_planning_EXPORTED_TARGETS}
)

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
   cfg/MotionPlanning.cfg
)