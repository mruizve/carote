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
add_library( model model/model.cpp model/joints.cpp model/kinematics.cpp )
target_link_libraries( model ${catkin_LIBRARIES} )

# controller library
add_library( controller
	controller/callbacks.cpp
	controller/commands.cpp
	controller/controller.cpp
	controller/goal.cpp
	controller/ros.cpp
	controller/poses.cpp
)
add_dependencies( controller carote_msgs_generate_messages_cpp )
target_link_libraries( controller model ${catkin_LIBRARIES} )

# control tools
add_executable( sleep tools/home.cpp )
add_dependencies( sleep carote_msgs_generate_messages_cpp )
target_link_libraries( sleep controller ${catkin_LIBRARIES} )

add_executable( wakeup tools/work.cpp )
add_dependencies( wakeup carote_msgs_generate_messages_cpp )
target_link_libraries( wakeup controller ${catkin_LIBRARIES} )

# controllers
add_executable( follower controllers/node.cpp controllers/follower.cpp )
add_dependencies( follower ${PROJECT_NAME}_gencfg carote_msgs_generate_messages_cpp )
target_compile_definitions( follower PRIVATE FOLLOWER=0 )
target_link_libraries( follower controller ${catkin_LIBRARIES} )

add_executable( positioner controllers/node.cpp controllers/positioner.cpp )
add_dependencies( positioner ${PROJECT_NAME}_gencfg carote_msgs_generate_messages_cpp )
target_compile_definitions( positioner PRIVATE POSITIONER=0 )
target_link_libraries( positioner controller ${catkin_LIBRARIES} )
