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
  State.msg
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
    src/DecoupledLQR.cc
    src/ControlDispatch.cc)
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
