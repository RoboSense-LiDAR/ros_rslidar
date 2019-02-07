add_library(rslidar_input input.cc)
target_link_libraries(rslidar_input
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})

add_library(rslidar_driver rsdriver.cpp)
target_link_libraries(rslidar_driver
        rslidar_input
        ${catkin_LIBRARIES})

# build the nodelet version
add_library(rsdriver_nodelet nodelet.cc rsdriver.cpp)
target_link_libraries(rsdriver_nodelet
        rslidar_input
        ${catkin_LIBRARIES}
        )

add_executable(rslidar_node rslidar_node.cpp)

if (catkin_EXPORTED_TARGETS)
    add_dependencies(rslidar_input ${catkin_EXPORTED_TARGETS})
endif ()

target_link_libraries(rslidar_node
        rslidar_driver
        rslidar_input
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        )

install(TARGETS rslidar_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
        )
install(TARGETS rsdriver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS rslidar_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS rslidar_driver
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
