/*
 *  Copyright (C) 2018-2020 Robosense Authors
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new rslidar_rawdata::RawData())
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("RS16"));

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  private_nh.param("output_points_topic", output_points_topic, std::string("rslidar_points"));
  output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe to rslidarScan packets
  std::string input_packets_topic;
  private_nh.param("input_packets_topic", input_packets_topic, std::string("rslidar_packets"));
  rslidar_scan_ = node.subscribe(input_packets_topic, 10, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));
  out_points_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
//  ROS_INFO("[cloud][convert] Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  out_points_->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  out_points_->header.frame_id = scanMsg->header.frame_id;
  out_points_->clear();
  if (model == "RS16")
  {
    out_points_->height = 16;
    out_points_->width = 24 * (int)scanMsg->packets.size();
    out_points_->is_dense = false;
    out_points_->resize(out_points_->height * out_points_->width);
  }
  else if (model == "RS32" || model == "RSBPEARL" || model == "RSBPEARL_MINI")
  {
    out_points_->height = 32;
    out_points_->width = 12 * (int)scanMsg->packets.size();
    out_points_->is_dense = false;
    out_points_->resize(out_points_->height * out_points_->width);
  }

  // process each packet provided by the driver

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], out_points_);
  }

  pcl::toROSMsg(*out_points_, out_msg_);

  output_.publish(out_msg_);
}
}  // namespace rslidar_pointcloud
