cmake_minimum_required(VERSION 2.8.3)
project(multilidar_timesync)
find_package(catkin REQUIRED roscpp roslib std_msgs sensor_msgs rslidar_msgs message_filters)
catkin_package()

include_directories(include
    ${catkin_INCLUDE_DIRS})

add_executable(timesync_2lidar_node src/timesync_2lidar.cpp)
add_dependencies(timesync_2lidar_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(timesync_2lidar_node
	${catkin_LIBRARIES}
)

add_executable(timesync_3lidar_node src/timesync_3lidar.cpp)
add_dependencies(timesync_3lidar_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(timesync_3lidar_node
	${catkin_LIBRARIES}
)



