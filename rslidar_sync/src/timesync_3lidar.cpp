#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/TimeReference.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher g_skippackets_num_pub1;
ros::Publisher g_skippackets_num_pub2;
ros::Publisher g_skippackets_num_pub3;
ros::Publisher g_maxnum_diff_packetnum_pub;
int g_packet_duration_us = 1200;
bool comparePair(const std::pair<unsigned, double> a, const std::pair<unsigned, double> b)
{
  return a.second < b.second;
}

void scanCallback(const sensor_msgs::TimeReference::ConstPtr& scan_msg1,
                  const sensor_msgs::TimeReference::ConstPtr& scan_msg2,
                  const sensor_msgs::TimeReference::ConstPtr scan_msg3)
{
  // calculate the first packet timestamp difference (us)
  double time1 = scan_msg1->header.stamp.sec * 1e6 + scan_msg1->header.stamp.nsec * 0.001;
  double time2 = scan_msg2->header.stamp.sec * 1e6 + scan_msg2->header.stamp.nsec * 0.001;
  double time3 = scan_msg3->header.stamp.sec * 1e6 + scan_msg3->header.stamp.nsec * 0.001;

  std::vector<std::pair<unsigned, double> > lidar_vector;
  lidar_vector.push_back(std::make_pair(1, time1));
  lidar_vector.push_back(std::make_pair(2, time2));
  lidar_vector.push_back(std::make_pair(3, time3));

  std::sort(lidar_vector.begin(), lidar_vector.end(), comparePair);

  unsigned skip_big = (lidar_vector[2].second - lidar_vector[0].second) / g_packet_duration_us;
  unsigned skip_small = (lidar_vector[1].second - lidar_vector[0].second) / g_packet_duration_us;

  std_msgs::String msg;
  std::stringstream ss;
  ss << "sync diff packets: " << skip_big;
  msg.data = ss.str();
  g_maxnum_diff_packetnum_pub.publish(msg);

  std_msgs::Int32 skip_num;
  if (skip_big > 1 && skip_small > 0)
  {
    // oldest skip 2 packets
    skip_num.data = 2;
    if (lidar_vector[0].first == 1)
    {
      g_skippackets_num_pub1.publish(skip_num);
    }
    else if (lidar_vector[0].first == 2)
    {
      g_skippackets_num_pub2.publish(skip_num);
    }
    else if (lidar_vector[0].first == 3)
    {
      g_skippackets_num_pub3.publish(skip_num);
    }

    // middle skip 1 packets
    skip_num.data = 1;
    if (lidar_vector[1].first == 1)
    {
      g_skippackets_num_pub1.publish(skip_num);
    }
    else if (lidar_vector[1].first == 2)
    {
      g_skippackets_num_pub2.publish(skip_num);
    }
    else if (lidar_vector[1].first == 3)
    {
      g_skippackets_num_pub3.publish(skip_num);
    }
  }
  else if (skip_big > 0 && skip_small == 0)
  {
    // oldest skip 2 packets
    skip_num.data = 1;
    if (lidar_vector[0].first == 1)
    {
      g_skippackets_num_pub1.publish(skip_num);
    }
    else if (lidar_vector[0].first == 2)
    {
      g_skippackets_num_pub2.publish(skip_num);
    }
    else if (lidar_vector[0].first == 3)
    {
      g_skippackets_num_pub3.publish(skip_num);
    }

    // middle skip 1 packets
    skip_num.data = 1;
    if (lidar_vector[1].first == 1)
    {
      g_skippackets_num_pub1.publish(skip_num);
    }
    else if (lidar_vector[1].first == 2)
    {
      g_skippackets_num_pub2.publish(skip_num);
    }
    else if (lidar_vector[1].first == 3)
    {
      g_skippackets_num_pub3.publish(skip_num);
    }
  }
  else if (skip_small > 0)
  {
    skip_num.data = 1;
    if (lidar_vector[0].first == 1)
    {
      g_skippackets_num_pub1.publish(skip_num);
    }
    else if (lidar_vector[0].first == 2)
    {
      g_skippackets_num_pub2.publish(skip_num);
    }
    else if (lidar_vector[0].first == 3)
    {
      g_skippackets_num_pub3.publish(skip_num);
    }
    else
    {
      ROS_ERROR("[sync-3] Oldest lidar skip one packet!");
//      std::cerr << "Oldest lidar skip one packet!" << std::endl;
    }
  }
  if (skip_big == 0)
  {
    // std::cout << "3 lidar synchronizer!" << std::endl;
  }
  // std::cout << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rs_sync");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // get the topic parameters
  std::string scan1_topic("/left/sync_header");
  std::string scan2_topic("/right/sync_header");
  std::string scan3_topic("/middle/sync_header");
  if (!nh_private.getParam(std::string("scan1_topic"), scan1_topic))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get scan1_topic, use the default scan1_topic: "<<scan1_topic);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] scan1_topic: "<<scan1_topic);
  }
  if (!nh_private.getParam(std::string("scan2_topic"), scan2_topic))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get scan2_topic, use the default scan2_topic: "<<scan2_topic);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] scan2_topic: "<<scan2_topic);
  }
  if (!nh_private.getParam(std::string("scan3_topic"), scan3_topic))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get scan3_topic, use the default scan3_topic: "<<scan3_topic);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] scan3_topic: "<<scan3_topic);
  }

  std::string skippackets1_topic("/left/skippackets_num");
  std::string skippackets2_topic("/right/skippackets_num");
  std::string skippackets3_topic("/middle/skippackets_num");
  if (!nh_private.getParam(std::string("skippackets1_topic"), skippackets1_topic))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get skippackets1_topic, use the default skippackets1_topic: "<<skippackets1_topic);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] skippackets1_topic: "<<skippackets1_topic);
  }
  if (!nh_private.getParam(std::string("skippackets2_topic"), skippackets2_topic))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get skippackets2_topic, use the default skippackets2_topic: "<<skippackets2_topic);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] skippackets2_topic: "<<skippackets2_topic);
  }
  if (!nh_private.getParam(std::string("skippackets3_topic"), skippackets3_topic))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get skippackets3_topic, use the default skippackets3_topic: "<<skippackets3_topic);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] skippackets3_topic: "<<skippackets3_topic);
  }

  bool is_all_rs32_lidars = false;
  if (!nh_private.getParam(std::string("is_all_rs32_lidars"), is_all_rs32_lidars))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get is_all_rs32_lidars, use the default is_all_rs32_lidars: " << is_all_rs32_lidars);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] The is_all_rs32_lidars: " << is_all_rs32_lidars);
  }
  if (is_all_rs32_lidars)
  {
    g_packet_duration_us = 600;
  }
  else
  {
    g_packet_duration_us = 1200;
  }

  // sync the rslidarscan
  message_filters::Subscriber<sensor_msgs::TimeReference> scan_sub1(nh, scan1_topic, 1);
  message_filters::Subscriber<sensor_msgs::TimeReference> scan_sub2(nh, scan2_topic, 1);
  message_filters::Subscriber<sensor_msgs::TimeReference> scan_sub3(nh, scan3_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::TimeReference, sensor_msgs::TimeReference,
                                                          sensor_msgs::TimeReference>
      ScanSyncPolicy;
  message_filters::Synchronizer<ScanSyncPolicy> scan_sync(ScanSyncPolicy(10), scan_sub1, scan_sub2, scan_sub3);
  scan_sync.registerCallback(boost::bind(&scanCallback, _1, _2, _3));

  g_skippackets_num_pub1 = nh.advertise<std_msgs::Int32>(skippackets1_topic.c_str(), 1, true);
  g_skippackets_num_pub2 = nh.advertise<std_msgs::Int32>(skippackets2_topic.c_str(), 1, true);
  g_skippackets_num_pub3 = nh.advertise<std_msgs::Int32>(skippackets3_topic.c_str(), 1, true);

  std::string sync_packet_diff_topic("/sync_packet_diff");
  if (!nh_private.getParam(std::string("sync_packet_diff_topic"), sync_packet_diff_topic))
  {
    ROS_ERROR_STREAM("[sync-3] Can't get sync_packet_diff_topic, use the default sync_packet_diff_topic: "
              <<sync_packet_diff_topic);
  }
  else
  {
    ROS_INFO_STREAM("[sync-3] sync_packet_diff: "<<sync_packet_diff_topic);
  }
  g_maxnum_diff_packetnum_pub = nh.advertise<std_msgs::String>(sync_packet_diff_topic, 1, true);

  ros::spin();
  return 0;
}
