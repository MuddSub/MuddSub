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
  rospack
  roslib
)

find_package(PythonLibs 2.7 REQUIRED)

find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs
	DEPENDS system_lib
)

message(${PROJECT_SOURCE_DIR}/../thirdparty)
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../thirdparty
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/Run.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})