# compile with c++11
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11" )

# target tracker
add_executable( target target/node.cpp target/target.cpp )
add_dependencies( target ${PROJECT_NAME}_gencfg )
target_link_libraries( target filtering ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} )

# simple follower
add_executable( follower follower/node.cpp follower/follower.cpp )
add_dependencies( follower ${PROJECT_NAME}_gencfg )
target_link_libraries( follower ${catkin_LIBRARIES} )

# utils library
file( GLOB Filtering_SRC filters/*.cpp )
add_library( filtering SHARED ${Filtering_SRC} )
target_link_libraries( filtering ${catkin_LIBRARIES} )

# tweak library
add_executable( tweak tweak/tweak.cpp tweak/node.cpp )
target_link_libraries( tweak ${catkin_LIBRARIES} )
