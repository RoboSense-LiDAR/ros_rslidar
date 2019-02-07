cmake_minimum_required(VERSION 2.8.3)
project(rslidar_pointcloud)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo
set(${PROJECT_NAME}_CATKIN_DEPS
        angles
        nodelet
        pcl_ros
        roscpp
        roslib
        sensor_msgs
        tf
        rslidar_driver
        rslidar_msgs
        dynamic_reconfigure
        )

find_package(catkin REQUIRED COMPONENTS
        ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)
find_package(Boost COMPONENTS signals)
find_package(PkgConfig REQUIRED)

generate_dynamic_reconfigure_options(cfg/CloudNode.cfg)

catkin_package(
        CATKIN_DEPENDS
        angles
        nodelet
        pcl_ros
        roscpp
        roslib
        sensor_msgs
        tf
        rslidar_driver
        rslidar_msgs
        dynamic_reconfigure
        LIBRARIES rslidar_data)

include_directories(${catkin_INCLUDE_DIRS}
        ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
        )


add_subdirectory(src)

install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

install(DIRECTORY rviz_cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz_cfg)