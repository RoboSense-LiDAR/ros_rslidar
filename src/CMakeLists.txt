add_library(rslidar_input input.cc)
target_link_libraries(rslidar_input 
                      ${catkin_LIBRARIES}
                      ${libpcap_LIBRARIES})

add_library(rslidar_driver modify_driver.cpp)
target_link_libraries(rslidar_driver
                      rslidar_input
                      ${catkin_LIBRARIES})

add_executable(fullscan fullscan_puber.cpp)
#add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(fullscan 
	rslidar_driver
        rslidar_input
	${catkin_LIBRARIES}
	${libpcap_LIBRARIES}
)

 #install(TARGETS my_signal
  #       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
