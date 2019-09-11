# RSLidar ROS 驱动说明

### 1. 依赖
* 1.安装ubuntu系统的电脑，推荐Ubuntu14.04或Ubuntu16.04，不要使用虚拟机
* 2.安装 ROS full-desktop版本。建议安装Indigo或Kinect
* 3.安装libpcap-dev

### 2. 安装和使用
* 1.将这个rslidar驱动拷贝到ROS工作区，例如“~/catkin_ws/src"
* 2.设置文件访问属性
```
cd ~/catkin_ws/src/ros_rslidar/rslidar_drvier
chmod 777 cfg/*
cd ~/catkin_ws/src/ros_rslidar/rslidar_pointcloud
chmod 777 cfg/*
```
* 3.编译和安装
```
cd ~/catkin_ws
catkin_make
```

* 4.设置PC ip  
默认情况下，RSLIDAR雷达设备ip设置为**192.168.1.200**，而PC ip则设置为**192.168.1.102**；同时默认**MSOP**端口为**6699**，**DIFOP**端口为**7788**. 所以需将PC ip设置为静态ip **192.168.1.102**才能和雷达通信。

* 5.以node方式运行  
这里提供示例launch文件，可以运行launch文件来观看点云数据。观看RS-LIDAR-16实时点云数据例子操作如下：
   
   * 1)打开一个终端并执行以下命令
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rslidar_pointcloud rs_lidar_16.launch
```
   * 2)新开一个终端并执行命令：
```
rviz
```
然后设置Fixed Frame为**“rslidar”**，接着添加Pointcloud2 类型和设置topic为**“rslidar_points"**。

* 6.以nodelet方式运行  
同时支持driver node和cloud node以nodelet方式运行。打开终端并执行以下命令：
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rslidar_pointcloud cloud_nodelet.launch
```
然后运行“rviz”来观看点云。

* 7.雷达校准参数  
**“rslidar_pointcloud/data”**目录下，存放雷达的校准参数文件。通过默认launch文件“rs_lidar_16.launch”加载以下3个文件：
   
 * rslidar_pointcloud/data/rs_lidar_16/angle.csv
 * rslidar_pointcloud/data/rs_lidar_16/ChannelNum.csv
 * rslidar_pointcloud/data/rs_lidar_16/curves.csv.

 如果要新增加雷达，可在**“rslidar_pointcloud/data/”**新建子目录，然后将校准参数文件放到子目录，接着修改launch文件，启动雷达雷达即可。launch文件修改可参考two_lidar.launch文件。

 **2018.09.01**之后在launch文件，可实现设置MSOP和DIFOP的端口。MSOP端口用于接收点云数据；而DIFOP端口用于接收雷达信息数据。如果DIFOP端口设置正确，可通过DIFOP数据包获取雷达的校准参数，而不用再通过校准文件，那可以忽略本地的雷达校准文件。如何检查DIFOP端口，可参考RS_LiDAR用户手册的附录G章节。

### 3.参数说明
雷达有一些参数需要设置，需要通过launch文件进行传递。
* model: 指定雷达类型。目前三个选项 RS32、RS16 和RSBPEARL，分别指32线、16线、补盲雷达
* device_ip：雷达ip
* msop_port：MSOP数据包接收端口
* difop_port：DIFOP数据包接收端口
* lidar_param_path：雷达标定参数文件存放位置
* pcap：要解析的pcap包存放路径。注意指定此参数，则驱动就不会接收从雷达发出的数据
* curves_path：指定反射率标定文件curves.csv完整路径。针对intensity_mode为1和2。
* angle_path：指定角度标定文件angle.csv完整路径
* channel_path：指定距离补偿文件ChannelNum.csv完整路径。一般使用默认全0参数即可。
* curves_rate_path：指定反射率洗漱标定文件CurveRate.csv完整路径。一般不需要
* max_distance：指定点云距离上限
* min_distance：指定点云距离下限
* resolution_type：指定雷达距离解析分辨类型，"0.5cm" 和 "1cm" 二选一
* intensity_mode：指定雷达反射率模式，分别为1,2,3 三选一
* cut_angle：采用角度分帧方式