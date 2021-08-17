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
  geometry_msgs
  message_generation
)
catkin_python_setup()
add_message_files(
 FILES
 Obstacle.msg
 Map.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 nav_msgs
 geometry_msgs
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


#add_executable(slam
#  src/RunFastSLAM.cc
#  src/FastSLAM.cc
#  src/DataLoader.cc
#	src/Particle.cc
#  src/EKF.cc
#)

#target_link_libraries(slam
#  ${catkin_LIBRARIES}
#  ${PYTHON_LIBRARIES})
#target_include_directories(slam
#  PRIVATE ${PYTHON_INCLUDE_DIRS}
#)
#add_dependencies(slam
#	${${PROJECT_NAME}_EXPORTED_TARGETS}
#	${catkin_EXPORTED_TARGETS}
#)

#add_executable(SLAMEvalServer
#  src/SLAMEvalServer.cc
#  src/FastSLAM.cc
#  src/DataLoader.cc
#	src/Particle.cc
#  src/EKF.cc
#)
#target_link_libraries(SLAMEvalServer ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
#target_include_directories(SLAMEvalServer PRIVATE ${PYTHON_INCLUDE_DIRS})
#add_dependencies(SLAMEvalServer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#catkin_install_python(PROGRAMS scripts/SLAMExampleNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} ) 