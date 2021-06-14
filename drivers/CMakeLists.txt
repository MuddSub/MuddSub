cmake_minimum_required(VERSION 3.0.2)
project(drivers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  imu_vn_100
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Depth.msg
  DVL.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drivers
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(DepthSensorExampleNode
	src/DepthSensorExampleNode.cc
  src/DepthSensorPublisher.cc
)
add_dependencies(DepthSensorExampleNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(DepthSensorExampleNode ${catkin_LIBRARIES})
