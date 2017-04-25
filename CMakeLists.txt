cmake_minimum_required(VERSION 2.8.3)
project(rslidar)

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
)

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS 
	     ${${PROJECT_NAME}_CATKIN_DEPS} 
             pcl_conversions
	     pcl_ros
	     roscpp 
	     rospy 
	     std_msgs
             genmsg
             cv_bridge
             message_generation
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)


include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} 
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)
include_directories(include ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/rslidarNode.cfg)

add_message_files(
  FILES
  rslidarPic.msg
  rslidarPacket.msg
  rslidarScan.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    DEPENDS yaml-cpp python-yaml
    INCLUDE_DIRS include
    LIBRARIES rslidar_rawdata rslidar_input
    CATKIN_DEPENDS message_runtime std_msgs
    )

#add_subdirectory(src/fullscan)
#add_subdirectory(src/segmentation)
add_subdirectory(src)


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
