^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rslidar_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2019-03-22)
------------------
* Fixes to packages and missing install commands
* Added install commands
* Fixed conflict with velodyne when installing on shared space
* renamed driver_nodelet to rsdriver_nodelet
* Fixed CMake, packages and Launch files
* Merge branch 'cut_angle_and_difop' into develop-curves-function
* bug: fix scan stamp mistake npackets - 1 to back in cut angle
* Merge branch 'develop-curves-function' of 192.168.1.20:system_group/ros_rslidar into develop-curves-function
* perf:improve the ordered pointcloud code
* fix:change "velodyne" to "rslidar"
* refactor:rename port to msop_portRename the port to msop_portDelete the uncessary debug information
* style: clang format code files and less cmake warning
* perf: set CMAKE_BUILD_TYPE Release, cpu drop from 28% to 19%
* feat: add the cut at specific angle featureIf the cut_angle is set to (0-2pi), the cut at specific angle featureactivated, otherwise use the fixed packets number mode.
* feat: add nodelet version
* feat: add difop parse
* Fix the bug that add in the CMakelist
* Merge pull request #5 <https://github.com/CPFL/robosense/issues/5>_ from enwaytech/ml/fix_cmake_warningsml/fix cmake warnings
* [ros_rslidar] fixed cmake warnings, removed unneccessary deps
* Merge pull request #1 <https://github.com/CPFL/robosense/issues/1>_ from RoboSense-LiDAR/develop-curves-functionMerge latest robosense updates
* Remove the uncessary dependency in package.xml
* Merge pull request #2 <https://github.com/CPFL/robosense/issues/2>_ from enwaytech/fix_cmake_installremove wrong install instructions in CMakeLists
* remove wrong install instructions in CMakeLists
* Modify the readme.Add the rviz startup in launch file.
* 1. Covert double to float2. Remove the re-defined variable.
* Reformate the code style.
* 1. Modify the code expression.2. Remove the unnecessary variables.
* Modify the parameter for the type of lidar in the launch file and add the intensity into display of RS-32.
* Merge RS-16 and RS-32 code.
* formate the code in input.cc
* Modify the ip to device ip
* Add send message function when poll out
* Fix the unpack logic to avoid cloud to be empty
* Add the diagnostic_updater into build depend to avoid someone occur the compile error
* Fix the wrong delete the rslidarScan header file
* Delete double set header files
* Add the license declaration.
* Modify the CMakeLists to avoid compile error and remove some uncessary definition.
* Clear the code
* Delete the empty include directory
* Remove the uncessary PWR file content
* Add inlcude directory
* Remove the uncessary network configuration
* Add mutil lidar function
* Clear unnecessary variables.
* rslidar mutil node version 1.0.0
* Contributors: Bo Chen, Lizhongpeng, Matthias Loebach, Tony Zhang, amc-nu, baoxianzhang, guoleiming, zhangbaoxian
