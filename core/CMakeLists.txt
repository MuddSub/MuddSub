cmake_minimum_required(VERSION 3.0.2)
project(core)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
	geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(state_publisher
	src/StatePublisher.cc
	src/OdomTfBroadcaster.cc
)
target_link_libraries(state_publisher ${catkin_LIBRARIES})


add_executable(urdf_parser
	src/URDFParser.cc
)
target_link_libraries(urdf_parser ${catkin_LIBRARIES})
