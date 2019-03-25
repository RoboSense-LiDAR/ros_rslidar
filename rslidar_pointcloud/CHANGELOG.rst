^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rslidar_pointcloud
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2019-03-25)
------------------
* Added license to source files
* Contributors: amc-nu

1.0.1 (2019-03-22)
------------------
* Fixes to packages and missing install commands
* install commands
* renamed conflicts on install space
* Added missing install commands
* Fixed conflict with velodyne when installing on shared space
* Merge https://github.com/RoboSense-LiDAR/ros_rslidar into develop-curves-function
* Merge branch 'develop-curves-function' of 192.168.1.20:system_group/ros_rslidar into develop-curves-function
* fix: the bug for dis_resolution_mode syntax
* feat: distance threshold as parameter pass by launch file
* fix: distance resolution type judgement error
* feat: add the default CurvesRate value 1.0
* feat: add 0.5cm support for RS16
* feat:add the 0.5cm resolution support
* feat: select the intensity mode into difop
* feat:add intensity factor
* feat: add intensity mode two
* fix: the parameter calculation should be 0.0001
* Renamed cloud_nodelet to rscloud_nodelet
* cloud_node renamed to rscloud_node, to play nice with other cloud_nodes
* Fixed CMake, packages and Launch files
* Merge branch 'cut_angle_and_difop' into develop-curves-function
* bug: fix scan stamp mistake npackets - 1 to back in cut angle
* fix: fix the minus missing bug in processDifop
* fix:the intensity calculation
* Merge branch 'develop-curves-function' of 192.168.1.20:system_group/ros_rslidar into develop-curves-function
* perf:improve the ordered pointcloud code
* fix:change "velodyne" to "rslidar"
* style: change the file mode
* refactor: Modify the readme
* fix: bug for intensity caculation
* refactor:rename port to msop_port
* fix: change difop data byte to both consider 0x00 and 0x ff
* style: clang format code files and less cmake warning
* perf: set CMAKE_BUILD_TYPE Release, cpu drop from 28% to 19%
* fix: fix check difop region not include 0xff case bug
* feat: add the cut at specific angle feature
* feat: add nodelet version
* fix: fix angle_flag from wrong position bug
* feat: add difop parse
* Delete the unnecessary debug output when load config files.
* Add the temperature into the old calIntensity function.
* add curves
* Merge pull request from RoboSense-LiDAR/develop-curves-function
* Remove the unnecessary dependency in package.xml
* Comment the judgement for ChannelNum colums. Do not use sigle columns.
* Remove round function in the calculation of intensity.
* Add curvesRate function for RS-LiDAR-32
* Remove a annotation.
* Add temperature compensation.
* Upload again for the last commit.
* Fix the bug for read parameter for curves function.
* Remove an output.
* Add curves function.
* Change the temperature range for RS32 from 31-71 to 31-81.
* Fix the build bug for catkin build.
* Remove an output.
* Add curves function.
* Modify the readme.
* Modify the readme to compatible with the new code structrue.
* 1. Covert double to float 2. Remove the re-defined variable.
* Remove unnecessary log information
* Reformate the code style.
* Modify the 16 beams and 32 beams unpack function logic to simplify the code.
* 1. Modify the code expression. 2. Remove the unnecessary variables.
* Merge branch 'develop-RS16-RL32' of 192.168.1.20:system_group/ros_rslidar into develop-RS16-RL32
* Delet unnecessary parameters.
* Adjust the file structure to compitable with 16 beams and 32 beams
* Fix the bug in the last commit.
* Fix the bug in the calculation of azimuth interpolation.
* Fix the bug of abnormal display color for intensity when the original intensity value is zero.
* Modify the parameter for the type of lidar in the launch file and add the intensity into display of RS-32.
* Merge RS-16 and RS-32 code.
* Change the temperature 70 to 71. Replace the default file on lidar1 and lidar2 directory to all 0.
* Change the temperature from 30 to 31
* 1. Comment the debug log for temperature 2. Replace the default ChanneNum to 41 colunm 0.
* Ingnore the abnormal angle packet. Remove the ChannleNum column judge
* Add device ip parameters in the launch file
* Remove the opencv
* update temperature per 20000 packets
* add temperature version
* 1. Modify the configuration path setting 2. Modify the launch file according to the new path setting
* Modify the max distance to 180m
* Change the X&Y axis direction.
* Modify the intensity caculate formula to compitable with rslidar-16 2.0
* Improve the code runtime
* Fix the unpack logic to avoid cloud to be empty
* code style fix
* Delete double set header files
* Updated the license declaration
* Add the license declaration.
* Modify the CMakeLists to avoid compile error and remove some unnecessary definition.
* Updated the launch file example
* Clear the code
* Delete the empty include directory
* Remove the unnecessary PWR file content
* Add inlcude directory
* Remove the unnecessary network configuration
* Add mutil lidar function
* Clear unnecessary variables.
* rslidar multi node version 1.0.0
* Contributors: Bo Chen, Lizhongpeng, Tony Zhang, amc-nu, baoxianzhang, guoleiming, songkan, zhangbaoxian, zhwu
