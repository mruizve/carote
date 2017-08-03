# compile with c++11
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11" )

# controller
add_executable( tweak tweak/node.cpp tweak/tweak.cpp tweak/listener.cpp tweak/publisher.cpp )
add_dependencies( tweak carote_msgs_generate_messages_cpp )
target_link_libraries( tweak ${catkin_LIBRARIES} )

# controller
add_library( controller
	controller/callbacks.cpp
	controller/controller.cpp
	controller/kinematics.cpp
	controller/ros.cpp
	controller/poses.cpp
)
add_dependencies( controller carote_msgs_generate_messages_cpp )
target_link_libraries( controller ${catkin_LIBRARIES} )

add_executable( follower follower/node.cpp follower/follower.cpp )
add_dependencies( follower ${PROJECT_NAME}_gencfg carote_msgs_generate_messages_cpp )
target_link_libraries( follower controller ${catkin_LIBRARIES} )
