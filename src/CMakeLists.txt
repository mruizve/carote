# compile with c++11
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11" )

# controller
add_executable( follower follower/node.cpp follower/follower.cpp )
add_dependencies( follower ${PROJECT_NAME}_gencfg )
target_link_libraries( follower ${catkin_LIBRARIES} )
