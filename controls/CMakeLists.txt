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
)

add_message_files(
  FILES
  ThrusterForceArray.msg
  ThrusterPWMArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs ct_core ct_optcon message_runtime
	DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/ControlsPubNode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulation_dynamics
              src/VehicleDynamics.cc
              src/DecoupledLQR.cc
              src/SimulationDynamics.cc)
add_dependencies(simulation_dynamics
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
	ct_core
	ct_optcon)
target_link_libraries(simulation_dynamics
  ${catkin_LIBRARIES}
)

add_executable(test_dynamics
              src/VehicleDynamics.cc
              src/DecoupledLQR.cc
              src/DynamicsTesting.cc)
add_dependencies(test_dynamics
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
	ct_core
	ct_optcon)
target_link_libraries(test_dynamics
  ${catkin_LIBRARIES}
)

add_executable(state_publisher
              src/StatePublisher.cc)
add_dependencies(state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_publisher
  ${catkin_LIBRARIES}
)

add_executable(ControlsPubNode
	src/ControlsPubNode.cc
  src/ControlsPublisher.cc
)
add_dependencies(ControlsPubNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ControlsPubNode ${catkin_LIBRARIES})
