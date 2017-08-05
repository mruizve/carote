# tweaks
add_executable( tweak-operator tweak/operator.cpp )
add_dependencies( tweak-operator carote_msgs_generate_messages_cpp )
target_link_libraries( tweak-operator ${catkin_LIBRARIES} )

add_executable( tweak-target tweak/target.cpp )
add_dependencies( tweak-target carote_msgs_generate_messages_cpp )
target_link_libraries( tweak-target ${catkin_LIBRARIES} )

# operator
add_executable( operator operator/operator.cpp operator/node.cpp )
add_dependencies( operator carote_msgs_generate_messages_cpp )
target_link_libraries( operator ${catkin_LIBRARIES} )

# model library
add_library( model model/model.cpp )
target_link_libraries( model ${catkin_LIBRARIES} )

# controller library
add_library( controller
	controller/callbacks.cpp
	controller/controller.cpp
	controller/ros.cpp
	controller/poses.cpp
)
add_dependencies( controller carote_msgs_generate_messages_cpp )
target_link_libraries( controller model ${catkin_LIBRARIES} )

# follower
add_executable( follower follower/node.cpp follower/follower.cpp )
add_dependencies( follower ${PROJECT_NAME}_gencfg carote_msgs_generate_messages_cpp )
target_link_libraries( follower controller ${catkin_LIBRARIES} )
