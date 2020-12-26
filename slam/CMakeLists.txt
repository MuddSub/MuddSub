cmake_minimum_required(VERSION 3.0.2)
project(slam)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  ct_core
  rospack
  Eigen3
  roslib
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
	DEPENDS system_lib

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(slam
  src/FastSLAM.cc
  src/DataLoader.cc
	src/Particle.cc
  src/EKF.cc
)

target_link_libraries(slam
  ${catkin_LIBRARIES})
target_include_directories(slam PUBLIC ${EIGEN3_INCLUDE_DIR})
add_dependencies(slam
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
