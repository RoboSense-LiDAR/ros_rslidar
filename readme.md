#### 1. Prerequisites
(1) Install a ubuntu PC. We suggested Ubuntu 14.04 or Ubuntu 16.04. Please do not use virtual machine.
(2) Install ros full-desktop version. We tried Indigo and Kinect.
(3) Please also install libpcap-dev.

####  2. Install
(1). Copy the whole rslidar package into a ROS workspace, i.e "~/catkin_ws/src".

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

#### 5. About the lidar configuration files
Under "**rslidar_pointcloud/data**" directory, you can find the lidar configuration data files for the exact sensor. By default the launch file "rs_lidar_16.launch" load the three files:
- rslidar_pointcloud/data/rs_lidar_16/angle.csv
- rslidar_pointcloud/data/rs_lidar_16/ChannelNum.csv
- rslidar_pointcloud/data/rs_lidar_16/curves.csv.

If you have more than one RSLIDAR, you can create new sub-directories under the "**rslidar_pointcloud/data/**", and put the data files into it.Then you need rewrite the launch file to start you lidar. We have put an example launch file "two_lidar.launch" to load two lidars together for reference.

#### 6. Only get just one laser data
In this modification version, we add a parameter in launch file that can specify just one laser data.
For 16 laser data, please still subscribe the "/rslidar_points" topic.
For the specified laser data, please still subscribe the "**/rslidar_points_signel**" topic.
The message type are both Pointcloud2. 
You can just run rs_lidar_16.launch to start the node. In the rs_lidar_16.launch, there is a parameter like below:
```
<param name="laser_channel" value="7"/>
```
You can change the value to eqaul to another number to set the laser data you want. About the relation ship between value and vertical number is as below:
| value | vetical angle |
|--------|--------|
|    0    |    -15 degree    |
|    1    |    -13 degree    |
|    2    |    -11 degree    |
|    3    |    -9 degree     |
|    4    |    -7 degree     |
|    5    |    -5 degree     |
|    6    |    -3 degree     |
|    7    |    -1 degree     |
|    8    |     15 degree    |
|    9    |     13 degree    |
|    10   |     11 degree    |
|    11   |     9 degree     |
|    12   |     7 degree     |
|    13   |     5 degree     |
|    14   |     3 degree     |
|    15   |     1 degree     |

