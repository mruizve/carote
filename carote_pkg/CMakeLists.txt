cmake_minimum_required(VERSION 2.8.3)
project(carote)

find_package(catkin REQUIRED COMPONENTS
	brics_actuator
	dynamic_reconfigure
	kdl_parser
	message_generation
	roscpp
	tf
	tf_conversions
	carote_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Follower.cfg
  cfg/Operator.cfg
  cfg/Positioner.cfg
)

catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES
	CATKIN_DEPENDS message_runtime carote_msgs
#	DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

# force release type when no type is defined
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release )
endif()

# extra compiler flags 
set( CMAKE_CXX_FLAGS "-Wall -Wextra" )
set( CMAKE_CXX_FLAGS_DEBUG "-g" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

subdirs( src )
