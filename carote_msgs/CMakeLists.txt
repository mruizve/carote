cmake_minimum_required(VERSION 2.8.3)
project(carote_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

add_message_files(
	FILES
	Operator.msg
	OperatorStamped.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
#	INCLUDE_DIRS include
	CATKIN_DEPENDS std_msgs
#	DEPENDS
)
