/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author yulinjun
 *  @author George
 */

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include <ros/ros.h>
#include <ros/package.h>
#include <rslidar_msgs/rslidarPic.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <rslidar_msgs/rslidarScan.h>
#include "myparam.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace rslidar_rawdata
{
    int    debug_a = 0;
    class RawData
    {
    public:
        RawData();
        ~RawData() {}

        void    init_setup();
        void    loadConfigFile();
        void    unpack(const rslidar_msgs::rslidarPacket &pkt,pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);
        float   pixelToDistance(int pixelValue, int passageway);

        float   calibrateIntensity(float inten,int calIdx,int distance);

    };


    cv::Mat mat_depth;// = cv::Mat::zeros(16,2100,cv::CV_32F); // 10HZ 2000 points per circle
    //cv::Mat mat_inten;// = cv::Mat::zeros(16,2100,cv::CV_U8C);
    float   VERT_ANGLE[16];
    //光衰信息表
    float   aIntensityCal[1600][16];
    //充能时间-实际功率对应表
    float   inPwrCurveDat[495];
    int     g_ChannelNum[16];



    void   removeOutlier(pcl::PointCloud<pcl::PointXYZI>::Ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr PCloudRemove(new pcl::PointCloud<pcl::PointXYZI>);
    
    rslidar_msgs::rslidarPic pic;


} // namespace velodyne_rawdata

#endif // __RAWDATA_H
