cmake_minimum_required(VERSION 3.0.2)
project(slam)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



 #find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/builds/cmake)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  ct_core
  rospack
  roslib
)

find_package(PythonLibs 2.7 REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
	DEPENDS system_lib
)

message(${PROJECT_SOURCE_DIR}/../thirdparty)
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../thirdparty
  ${catkin_INCLUDE_DIRS}
)


add_executable(slam
  src/FastSLAM.cc
  src/DataLoader.cc
	src/Particle.cc
  src/EKF.cc
)

target_link_libraries(slam
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES})
target_include_directories(slam
  PRIVATE ${PYTHON_INCLUDE_DIRS}
)
add_dependencies(slam
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

add_executable(SLAMEvalServer src/SLAMEvalServer.cc)
target_link_libraries(SLAMEvalServer ${catkin_LIBRARIES})
#add_dependencies(SLAMEvalServer slam_gencpp)
