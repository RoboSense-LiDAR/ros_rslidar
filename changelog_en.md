## RSLidar ROS Driver Changelog

### V1.0.0(2017-09-29)
---
first release for mutil node version

### V2.0.0(2017-11-23)
---
* 1.Add the temperature compensation for distance.
* 2.Remove the OpenCV code.

### V3.0.0(2018-05-10)
---
* 1.Add new feature: add the RS-LiDAR-32 support
* 2.Fix bug: modify the temperature compensation range to 30~71 degree centigrade.
* 3.Use the new code structure.
* 4.Add new feature: modify the intensity calculation formula.
* 5.Add new feature: compatible with the different calibration files.
* 6.Add new feature: add the multi lidar synchronization function.

### V3.1.0(2018-07-14)
---
* 1.Add new feature: Add temperature compensation in internsity caculation.
* 2.Remove uncessary dependency in package.xml

### V4.0.0(2019-04-11)
---
* 1.Add new feature: add new parameter name and value in launch file where max_distance is used to set maximum distance and min_distance is used to set minimum distance.
* 2.Add the difop packets parser code.
* 3.Add new feature: compatible with firmware which's output distance resolution is 0.5cm.
* 4.Fix bug: fix the wrong distance and coordination of point cloud when output distance resolution of firmware is 1cm.
* 5.Add new feature: compatible with firmware which's output intensity directly without calibration formula (intensity_mode = 3).
* 6.Add new feature: add 'Dual Return' packet parser support
* 7.Move timestamp getting code for multiple lidar 's synchronization from input.cc to rsdriver.cpp
* 8.Add 'Resolution' and 'intensity_mode' parameter into launch file so that it can compatible with the bag files that did not record the difop packets.
* 9.Fix bug: add coordinate transformation compensation into the XYZ calculation
* 10.Normalize the code
* 11.Add the horizontal angle framing code

### V4.1.0(2019-09-11)
---
* 1.Fix bug: fix the packet_rate calculation bug for different return mode.
* 2.Add the function for reading the calibrated angle from difop of RS32.
* 3.Update the log format to make it more clear to know the source of the information.
* 4.Change the trigonometric function calculation to look-up table, this can improve the code performance efficiency.
* 5.Change the coordinate transformation compensation for different lidars.
* 6.Change the scan split way to angle by default. Change the packet_rate value corresponding to 18K sampling rate.
* 7.Add RSBPEARL configuration support.

### V4.1.1(2019-11-29)
---
* 1. Fix bug: fix the angle parsing bug for BPearl
* 2. Fix bug: fix the angle difference error when we set the specific FOV of LiDAR
