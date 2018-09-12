add_library(rslidar_data rawdata.cc)
target_link_libraries(rslidar_data
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(rslidar_point convert.cc)
target_link_libraries(rslidar_point
    rslidar_data
    ${catkin_LIBRARIES})


add_library(rslidar_cloud_nodelet cloud_nodelet.cc)
target_link_libraries(rslidar_cloud_nodelet rslidar_point
                      ${catkin_LIBRARIES})

add_executable(rslidar_cloud_node cloud_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_data ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(rslidar_cloud_node
    rslidar_point
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})
