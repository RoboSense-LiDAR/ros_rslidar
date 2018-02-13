#### 1. Prerequisites
(1) Install a ubuntu PC. We suggested Ubuntu 14.04 or Ubuntu 16.04. Please do not use virtual machine.
(2) Install ros full-desktop version. We tried Indigo and Kinect.
(3) Please also install libpcap-dev.

####  2. Install
(1). Copy the whole rslidar directory into ROS workspace, i.e "~/catkin_ws/src".

(2). Check the file attributes:

```
cd ~/catkin_ws/src/ros_rslidar/rslidar_drvier
chmod 777 cfg/*
cd ~/catkin_ws/src/ros_rslidar/rslidar_pointcloud
chmod 777 cfg/*
```

(3). Then to compile the source code and to install it:

```
cd ~/catkin_ws
catkin_make
```
#### 3. Configure PC IP
By default, the RSLIDAR is configured to **192.168.1.200** as its device IP and **192.168.1.102** as PC IP that it would communicate. The default MSOP port is 6699.
So you need configure your PC IP as a static one **192.168.1.102**.

#### 4. Run and view real time data
We have provide example launch files under rslidar_pointcloud/launch, we can run the launch file to view the point cloud data. For example, if we want to view RS-LiDAR-16 real time data:
(1). Open a new terminal and run:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rslidar_pointcloud rs_lidar_16.launch
```

(2). Open a new terminal and run:

```
rviz
```
Set the Fixed Frame to "**rslidar**".
Add a Pointcloud2 type and set the topic to "**rslidar_points**".

#### 5. About the lidar data
Under "**rslidar_pointcloud/data**" directory, you can find the lidar data files for the exact sensor. By default the launch file "rs_lidar_16.launch" load the three files:
- rslidar_pointcloud/data/rs_lidar_16/angle.csv
- rslidar_pointcloud/data/rs_lidar_16/ChannelNum.csv
- rslidar_pointcloud/data/rs_lidar_16/curves.csv.

If you have more than one RSLIDAR, you can create new sub-directories under the "**rslidar_pointcloud/data/**", and put the data files into it.Then you need rewrite the launch file to start you lidar. We have put an example launch file "two_lidar.launch" to load two lidars together for reference.
