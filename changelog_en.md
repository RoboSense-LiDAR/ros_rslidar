## RSLidar ROS Driver Changelog

### V1.0.0(2018-12-03) 
---
first release for mutil node version

### V1.1.0(2019-03-18) 
---
Summary:
* 1.Add new feature: add new parameter name and value in launch file where max_distance is used to set maximum distance of display of 
point cloud and min_distance is for minimum distance.
* 2.fix bug: fix the wrong distance and coordination of point cloud when output distance resolution of firmware is 1cm.
* 3.Add new feature: compatible with firmware which's output distance resolution is 0.5cm.
* 4.Add new feature: Support to new mode of intensity(Mode3)
* 5.Add 'Dual Return' packet parser support
* 6.Move timestamp getting code for multiple lidar 's synchronization from input.cc to rsdriver.cpp


