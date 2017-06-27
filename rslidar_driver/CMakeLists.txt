cmake_minimum_required(VERSION 2.8.3)
project(rslidar_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    dynamic_reconfigure
    diagnostic_updater 
    rslidar_msgs
)

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS 
	${${PROJECT_NAME}_CATKIN_DEPS} 
    pcl_conversions 
	rospy 
	std_msgs
    genmsg
    cv_bridge
    message_generation
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)


include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} 
	${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

generate_dynamic_reconfigure_options(cfg/rslidarNode.cfg)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    DEPENDS yaml-cpp python-yaml
    CATKIN_DEPENDS message_runtime std_msgs
    )

add_subdirectory(src)
#add_subdirectory(src/lib)
#add_subdirectory(src/driver)


 #install(TARGETS my_signal
  #       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
install(PROGRAMS scripts/gen_calibration.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_rslidar.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(PROGRAMS src/vdump
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
