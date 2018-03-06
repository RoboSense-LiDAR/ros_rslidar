#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rslidar_msgs/rslidarPic.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <rslidar_msgs/rslidarScan.h>

ros::Publisher g_skippackets_num_pub1;
ros::Publisher g_skippackets_num_pub2;

void scanCallback(const rslidar_msgs::rslidarScan::ConstPtr &scan_msg1, const rslidar_msgs::rslidarScan::ConstPtr &scan_msg2)
{
  std::cerr << "Enter scanCallback function!" << std::endl;
  // calculate the first packet timestamp difference (us)
  // double time1 = scan_msg1->header.stamp.sec * 1e6 + scan_msg1->header.stamp.nsec * 0.001;
  // double time2 = scan_msg2->header.stamp.sec * 1e6 + scan_msg2->header.stamp.nsec * 0.001;
  double time1 = 0.001 * scan_msg1->header.stamp.nsec;
  double time2 = 0.001 * scan_msg2->header.stamp.nsec;

  double delta_time = time1 - time2;
  // std::cout << "delta time: " << delta_time << "us" << std::endl;
  // std::cout << "delta time: " << delta_time/1000 << "ms" << std::endl;
  int delta_skip    = delta_time / 1200;
  std::cout << "skip " << delta_skip << " packets!"<< std::endl;
  std_msgs::Int32 skip_num;
  if (delta_skip > 0)
  {
    std::cout << "lidar 2 faster, lidar 2 skip one packets!"<< std::endl;
    skip_num.data = 1;
    g_skippackets_num_pub2.publish(skip_num);
  }
  else if(delta_skip < 0)
  {
    std::cout << "lidar 1 faster, lidar 1 skip one packet!"<< std::endl;
    skip_num.data = 1;
    g_skippackets_num_pub1.publish(skip_num);
  }
  else
  {
    std::cout << "Synchronizer!" << std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multilidar_timesync");
  ros::NodeHandle nh("~");

  // get the topic parameters
  std::string scan1_topic;
  std::string scan2_topic;
  nh.getParam(std::string("scan1_topic"), scan1_topic);
  nh.getParam(std::string("scan2_topic"), scan2_topic);

  std::string skippackets1_topic;
  std::string skippackets2_topic;
  nh.getParam(std::string("skippackets1_topic"), skippackets1_topic);
  nh.getParam(std::string("skippackets2_topic"), skippackets2_topic);

  // sync the rslidarscan
  message_filters::Subscriber<rslidar_msgs::rslidarScan> scan_sub1(nh, scan1_topic.c_str(), 1);
  message_filters::Subscriber<rslidar_msgs::rslidarScan> scan_sub2(nh, scan2_topic.c_str(), 1);
  typedef message_filters::sync_policies::ApproximateTime<rslidar_msgs::rslidarScan, rslidar_msgs::rslidarScan> ScanSyncPolicy;
  message_filters::Synchronizer<ScanSyncPolicy> scan_sync(ScanSyncPolicy(10), scan_sub1, scan_sub2);
  scan_sync.registerCallback(boost::bind(&scanCallback, _1, _2));

  g_skippackets_num_pub1 = nh.advertise<std_msgs::Int32>(skippackets1_topic.c_str(), 1, true);
  g_skippackets_num_pub2 = nh.advertise<std_msgs::Int32>(skippackets2_topic.c_str(), 1, true);

  ros::spin();
  return 0;
}
