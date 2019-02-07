add_library(rslidar_data rawdata.cc)
target_link_libraries(rslidar_data
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})

add_library(rslidar_point convert.cc)
target_link_libraries(rslidar_point
        rslidar_data
        ${catkin_LIBRARIES})


add_library(rscloud_nodelet cloud_nodelet.cc)
target_link_libraries(rscloud_nodelet rslidar_point
        ${catkin_LIBRARIES})

add_executable(rscloud_node cloud_node.cc)

if (catkin_EXPORTED_TARGETS)
    add_dependencies(rslidar_data ${catkin_EXPORTED_TARGETS})
endif ()

target_link_libraries(rscloud_node
        rslidar_point
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})

install(TARGETS
        rslidar_data
        rscloud_nodelet
        rscloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )