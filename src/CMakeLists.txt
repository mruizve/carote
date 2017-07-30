# compile with c++11
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11" )

# controller
add_executable( tweak tweak/node.cpp tweak/tweak.cpp tweak/listener.cpp tweak/publisher.cpp )
target_link_libraries( tweak ${catkin_LIBRARIES} )

# controller
add_executable( controller controller/node.cpp controller/controller.cpp )
add_dependencies( controller ${PROJECT_NAME}_gencfg )
target_link_libraries( controller ${catkin_LIBRARIES} )
