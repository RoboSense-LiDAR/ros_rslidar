add_library(rslidar_data rawdata.cc )
target_link_libraries(rslidar_data
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(rslidar_point convert.cc )
target_link_libraries(rslidar_point
    rslidar_data
    ${catkin_LIBRARIES})

add_executable(cloud_node cloud_node.cc)
target_link_libraries(cloud_node
    rslidar_point
    rslidar_input
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    )
