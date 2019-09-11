#### 1. Prerequisites
(1) Install a ubuntu PC. We suggested Ubuntu 14.04 or Ubuntu 16.04. Please do not use virtual machine.
(2) Install ros full-desktop version. We tried Indigo and Kinect.
(3) Please also install libpcap-dev.

####  2. Install
(1). Copy the whole rslidar ROS driver directory into ROS workspace, i.e "~/catkin_ws/src".

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
By default, the RSLIDAR is configured to **192.168.1.200** as its device IP and **192.168.1.102** as PC IP that it would communicate. The default **MSOP port is 6699** while **DIFOP port is 7788**.
So you need configure your PC IP as a static one **192.168.1.102**.

#### 4. Run as independent node
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

#### 5. Run as nodelet
We can also run the driver node and cloud node as a nodelet.
Open a new terminal and run:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rslidar_pointcloud cloud_nodelet.launch
```
Then we can run view the pointcloud via "rviz"

#### 6. About the lidar calibration parameters
Under "**rslidar_pointcloud/data**" directory, you can find the lidar calibration parameters files for the exact sensor. By default the launch file "rs_lidar_16.launch" load the three files:
- rslidar_pointcloud/data/rs_lidar_16/angle.csv
- rslidar_pointcloud/data/rs_lidar_16/ChannelNum.csv
- rslidar_pointcloud/data/rs_lidar_16/curves.csv.

If you have more than one RSLIDAR, you can create new sub-directories under the "**rslidar_pointcloud/data/**", and put the data files into it.Then you need rewrite the launch file to start you lidar. We have put an example launch file "two_lidar.launch" to load two lidars together for reference.

**Begin from 2018.09.01**
In the launch file, we could set the MSOP port and DIFOP port.
MSOP port is used for receive the main point cloud data, while DIFOP port is used for receive the device information data. If we set the right DIFOP port, we will get the lidar calibration parameters from the DIFOP packets not from the files, so can ignore the local lidar calibration files. About how to check the DIFOP port, please reference the Appendix G in RS-LiDARR manual.

### 3.Parameters instruction
There are some parameters need to be set in the launch files for operating LiDAR correctly.
* model: Assign the LiDAR model.There are three selection:RS32,RS16 and RSBPEARL.
* device_ip：Set the LiDAR network ip address.
* msop_port：LiDAR MSOP network port number.
* difop_port：LiDAR DIFOP network port number
* lidar_param_path：The location for the calibration files.
* pcap：The location for the pcap which needs to be parse. After setting this parameter, the online connection for LiDAR will stop.
* curves_path：Assign the location for reflectivity calibration files. Just valid for intensity_mode=1 and intensity_mode=2.
* angle_path：Assign the location for angle calibration files.
* channel_path：Assign the location for distance calibration files.
* curves_rate_path：Assign the location for reflectivity coefficient. This can be set to null by default.
* max_distance：The upper limit for distance.
* min_distance：The lower limit for distance.
* resolution_type：The LSB for distance."0.5cm" or "1cm" .
* intensity_mode：The mode for reflectivity.1, 2 or 3.
* cut_angle：The scan split way for using angle.