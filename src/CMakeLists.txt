# target tracker
add_executable( target target/node.cpp target/target.cpp )
target_link_libraries( target filtering ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} )

# utils library
file( GLOB Filtering_SRC filters/*.cpp )
add_library( filtering SHARED ${Filtering_SRC} )
target_link_libraries( filtering ${catkin_LIBRARIES} )
