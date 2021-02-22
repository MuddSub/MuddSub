cmake_minimum_required(VERSION 3.0.2)
project(slam)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/builds/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  rospack
  roslib
)

find_package(PythonLibs 2.7 REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  Map.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# message(${PROJECT_SOURCE_DIR}/../thirdparty)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../thirdparty
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
	DEPENDS
)

add_executable(slam_example_node
	src/SLAMExampleNode.cc
  src/SLAMPublisher.cc
)
add_dependencies(slam_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_example_node ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
