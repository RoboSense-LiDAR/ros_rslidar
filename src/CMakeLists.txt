add_library(rslidar_input input.cc convertion.cc)
target_link_libraries(rslidar_input
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(rslidar_driver rsdriver.cpp )
target_link_libraries(rslidar_driver
    rslidar_input
    ${catkin_LIBRARIES})

add_executable(rslidar_node rslidar_node.cpp)

add_dependencies(rslidar_input rslidar_generate_messages_cpp)
target_link_libraries(rslidar_node
    rslidar_driver
    rslidar_input
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    )
