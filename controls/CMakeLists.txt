cmake_minimum_required(VERSION 3.0.2)
project(controls)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  ct_core
  ct_optcon
  tf2_geometry_msgs
)

add_message_files(
  FILES
  State.msg
  ThrusterForceArray.msg
  ThrusterPWMArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs ct_core ct_optcon message_runtime tf2_geometry_msgs
	DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/ControlsExampleNode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_dispatch
    src/DecoupledLQR.cc
    src/PID.cc
    src/VehicleDynamics.cc
    src/SixDegreePID.cc
    src/DecoupledLQR.cc
    src/ControlDispatch.cc
    src/Controller.cc)
add_dependencies(control_dispatch
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
	ct_core
	ct_optcon)
target_link_libraries(control_dispatch
  ${catkin_LIBRARIES}
)

add_executable(state_publisher
            src/StatePublisher.cc)
add_dependencies(state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_publisher
  ${catkin_LIBRARIES}
)

add_executable(ControlsExampleNode
	src/ControlsExampleNode.cc
  src/ControlsPublisher.cc
)
add_dependencies(ControlsExampleNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ControlsExampleNode ${catkin_LIBRARIES})
