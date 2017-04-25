#include <ros/ros.h>
#include "modify_driver.h"
#include "convert_fullscan.h"
#include "std_msgs/String.h"

int freq;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fullscan");
    ros::NodeHandle node;
    ros::NodeHandle node2;
    ros::NodeHandle private_nh("~");
    private_nh.param("frequency",freq,5);

    pic_output = node.advertise<rslidar::rslidarPic>("rawdata_fullscan",10);
    pc_output = node2.advertise<sensor_msgs::PointCloud2>("rslidar", 10);
    init_setup();
    // start the driver

    rslidar_driver::rslidarDriver dvr(node, private_nh);
    // loop until shut down or end of file
    while(ros::ok())
    {

        while(fullscan == 0)
        {
            dvr.poll();
        }
        fullscan = 0;
        ros::spinOnce();
    }

    return 0;

}
