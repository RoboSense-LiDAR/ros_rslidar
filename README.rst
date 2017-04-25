
1. 将程序包rslidar拷贝到ros工作目录下,如 ~/catkin_ws/src/
2. 进入ros工作目录,并编译
   $ cd ~/catkin_ws
   $ catkin_make
   $ source ~/catkin_ws/devel/setup.bash
3. 运行离线数据,如离线数据的路径为 /home/hostname/data/rslidar_0216.pcap
   $ roscore
   $ rosrun rslidar fullscan _pcap:="/home/sti/data/rslidar_0216.pcap"
   $ rviz
   设置rviz中Fixed Frame为 /rslidar,设置Topic为 rslidar
4. 运行实时数据
   设置本机IP为固定IP, IP地址为 192.168.1.102, 子码掩网为 255.255.0.0
   $ roscore
   $ rosrun rslidar fullscan
   $ rviz
   设置rviz中Fixed Frame为 /rslidar,设置Topic为 rslidar

