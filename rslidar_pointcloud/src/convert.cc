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
#include <eigen_conversions/eigen_msg.h>

namespace rslidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) :
    data_(new rslidar_rawdata::RawData()),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("RS16"));
  private_nh.param("compensate_motion", compensate_motion_, false);
  private_nh.param("fixed_frame", fixed_frame_, std::string(""));
  private_nh.param("tf_wait_time", tf_wait_time_, 0.02);

  ROS_INFO_STREAM("[cloud][convert] compensate motion: " << compensate_motion_);
  if (compensate_motion_) {
    if (fixed_frame_.empty()) {
      ROS_ERROR_STREAM("[cloud][convert] Motion compensation enabled but got empty fixed frame id");
    } else {
      ROS_INFO_STREAM("[cloud][convert] fixed frame: " << fixed_frame_);
      ROS_INFO_STREAM("[cloud][convert] Tf wait time: " << tf_wait_time_);
    }
  }

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
}

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
//  ROS_INFO("[cloud][convert] Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model == "RS32" || model == "RSBPEARL" || model == "RSBPEARL_MINI")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }

  // process each packet provided by the driver

  data_->block_num = 0;
  geometry_msgs::TransformStamped tf_comp_stamped;
  Eigen::Affine3d tf_comp_eigen;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    if (compensate_motion_) {
      // Set transform to identity, in case transform lookup fails.
      tf_comp_eigen.setIdentity();
      try{
        tf_comp_stamped = tf_buffer_.lookupTransform(scanMsg->header.frame_id, // Target frame.
                                                     scanMsg->header.stamp,  // Target time.
                                                     scanMsg->header.frame_id, // Source frame.
                                                     scanMsg->packets[i].stamp,  // Source time.
                                                     fixed_frame_, // Fixed frame.
                                                     ros::Duration(tf_wait_time_));  // Wait time for transform.
        ROS_DEBUG_STREAM(tf_comp_stamped);
        tf::transformMsgToEigen(tf_comp_stamped.transform, tf_comp_eigen);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
      }
      data_->unpack(scanMsg->packets[i], outPoints, tf_comp_eigen.cast<float>());
    } else {
      data_->unpack(scanMsg->packets[i], outPoints);
    }

  }
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);

  output_.publish(outMsg);
}
}  // namespace rslidar_pointcloud
