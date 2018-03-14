cmake_minimum_required(VERSION 2.8.3)
project(rslidar_pointcloud)

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
include_directories(${catkin_INCLUDE_DIRS} 
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)
    
#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#target_link_libraries(dynamic_reconfigure_node
#   ${catkin_LIBRARIES}
# )
generate_dynamic_reconfigure_options(cfg/CloudNode.cfg)
#add_subdirectory(src/lib)
#add_subdirectory(src/conversions)
add_subdirectory(src)

