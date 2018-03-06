cmake_minimum_required(VERSION 2.8.3)
project(multilidar_timesync)
find_package(catkin REQUIRED roscpp roslib std_msgs sensor_msgs rslidar_msgs message_filters)
catkin_package()

include_directories(include
    ${catkin_INCLUDE_DIRS})

add_executable(timesync_node src/timesync.cpp)
add_dependencies(timesync_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(timesync_node
	${catkin_LIBRARIES}
)

