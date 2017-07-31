# compile with c++11
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11" )

# controller
add_executable( tweak tweak/node.cpp tweak/tweak.cpp tweak/listener.cpp tweak/publisher.cpp )
target_link_libraries( tweak ${catkin_LIBRARIES} )

# controller
add_library( controller controller/controller.cpp controller/init.cpp controller/callbacks.cpp )
target_link_libraries( controller ${catkin_LIBRARIES} )

add_executable( follower follower/node.cpp follower/follower.cpp )
add_dependencies( follower ${PROJECT_NAME}_gencfg )
target_link_libraries( follower controller ${catkin_LIBRARIES} )
