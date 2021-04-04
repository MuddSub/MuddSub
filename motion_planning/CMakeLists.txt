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

add_executable(astar
	src/AStar.cc
)

target_link_libraries(
	astar
  ${catkin_LIBRARIES}
)
add_dependencies(
  astar
  ${motion_planning_EXPORTED_TARGETS}
)


