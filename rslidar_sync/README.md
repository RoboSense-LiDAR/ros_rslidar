# rslidar_sync multilidar time synchronization package

# How to use

+ Connect multiple lidars with the same GPS. The system time of the lidars will be the same with the GPS time.

+ Modify the lidar ip, port and namespace in the launch/rslidar_sync_nlidar.launch (n = 2 or 3). Please make sure the time_synchronization param value is true.

+ Build and launch the rslidar_sync_nlidar.launch.
```
roslaunch rslidar_sync rslidar_sync_2lidar.launch
```
or
```
roslaunch rslidar_sync rslidar_sync_3lidar.launch
```

+ Check the time synchronization result.
```
rostopic echo /sync_packet_diff
```
It shows like this, n < 4 says good time synchronization. n >= 4 says bad result.
```
sync diff packets: n
```
If it doesn't show any message, please check the GPS connection. You may also check the lidar timestamp below.
```
rostopic echo YOUR_LIDAR_TOPIC --noarr
```
