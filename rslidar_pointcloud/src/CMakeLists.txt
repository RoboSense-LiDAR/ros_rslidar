add_library(rslidar_data rawdata.cc)
target_link_libraries(rslidar_data
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(rslidar_point convert.cc)
target_link_libraries(rslidar_point
    rslidar_data
    ${catkin_LIBRARIES})


add_library(rcloud_nodelet  cloud_nodelet.cc)
target_link_libraries(rcloud_nodelet  rslidar_point
                      ${catkin_LIBRARIES})

add_executable(rcloud_node  cloud_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_data ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(rcloud_node 
    rslidar_point
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})
