/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud {
    /** @brief Constructor. */
    Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) :
            data_(new rslidar_rawdata::RawData()) {
        data_->loadConfigFile(private_nh);            //load lidar parameters
        data_->init_setup();

        std::string model;
        private_nh.param("model", model, std::string("RS16"));

        // advertise output point cloud (before subscribing to input data)
        output_ =
                node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 10);

        srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::
        CloudNodeConfig> >(private_nh);
        dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::
        CallbackType f;
        f = boost::bind(&Convert::callback, this, _1, _2);
        srv_->setCallback(f);

        // subscribe to rslidarScan packets
        rslidar_scan_ =
                node.subscribe("rslidar_packets", 10,
                               &Convert::processScan, (Convert *) this,
                               ros::TransportHints().tcpNoDelay(true));
    }

    void Convert::callback(rslidar_pointcloud::CloudNodeConfig &config,
                           uint32_t level) {
        ROS_INFO("Reconfigure Request");
        // config_.time_offset = config.time_offset;
    }

    /** @brief Callback for raw scan messages. */
    void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr &scanMsg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
        outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
        outPoints->header.frame_id = scanMsg->header.frame_id;
        // process each packet provided by the driver

        bool finish_packets_parse = false;
        for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
            if (i == (scanMsg->packets.size() - 1)) {
                // ROS_INFO_STREAM("Packets per scan: "<< scanMsg->packets.size());
                finish_packets_parse = true;
            }

            data_->unpack(scanMsg->packets[i], outPoints, finish_packets_parse);
        }
        sensor_msgs::PointCloud2 outMsg;
        pcl::toROSMsg(*outPoints, outMsg);

        //if(outPoints->size()==0){
        //    ROS_INFO_STREAM("Height1: "<<outPoints->height<<" Width1: "<<outPoints->width);
        //}
        output_.publish(outMsg);
    }
} // namespace rslidar_pointcloud
