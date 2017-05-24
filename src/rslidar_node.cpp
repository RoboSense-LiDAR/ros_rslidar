#include <ros/ros.h>
#include "rsdriver.h"
#include "std_msgs/String.h"


/**
 * ROS Main node entry point
 * @brief main  ros node of rsliar driver
 * @param argc
 * @param argv
 * @return
 */
using namespace rs_driver;
volatile sig_atomic_t flag=1;
static void my_handler(int sig)
{
	flag=0;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rsdriver");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

 	signal(SIGINT,my_handler);
 	
    // start the driver
    rs_driver::rslidarDriver dvr(node, private_nh);
    // loop until shut down or end of file
    while(ros::ok())
    {
        dvr.poll();
        ros::spinOnce();
    }

    return 0;

}
