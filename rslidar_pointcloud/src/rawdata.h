/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *	@author Tony Zhang
 */

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include <ros/ros.h>
#include <ros/package.h>
#include <rslidar_msgs/rslidarPic.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <rslidar_msgs/rslidarScan.h>
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace rslidar_rawdata
{   
	//static const float  ROTATION_SOLUTION_ = 0.18f;  //水平角分辨率 10hz
	static const int    POINT_PER_CIRCLE_ =  2000;
	static const int    DATA_NUMBER_PER_SCAN = 40000 ; //Set 40000 to be large enough
	static const int    SIZE_BLOCK = 100;
	static const int    RAW_SCAN_SIZE = 3;
	static const int    SCANS_PER_BLOCK = 32;
	static const int    BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE); //96

	static const float  ROTATION_RESOLUTION = 0.01f; /**< degrees 旋转角分辨率*/
	static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

	static const float  DISTANCE_MAX = 150.0f;        /**< meters */
	static const float  DISTANCE_MIN = 0.2f;        /**< meters */
    static const float  DISTANCE_RESOLUTION = 0.01f; /**< meters */
	static const float  DISTANCE_MAX_UNITS = (DISTANCE_MAX
		                                     / DISTANCE_RESOLUTION + 1.0);
	/** @todo make this work for both big and little-endian machines */
	static const uint16_t UPPER_BANK = 0xeeff; //
	static const uint16_t LOWER_BANK = 0xddff;


	/** Special Defines for RS16 support **/
	static const int    RS16_FIRINGS_PER_BLOCK =   2;
	static const int    RS16_SCANS_PER_FIRING  =  16;
	static const float  RS16_BLOCK_TDURATION   = 100.0f;   // [µs]
	static const float  RS16_DSR_TOFFSET       =   3.0f;   // [µs]
	static const float  RS16_FIRING_TOFFSET    =  50.0f;   // [µs]
	
	/** \brief Raw rslidar data block.
	 *
	 *  Each block contains data from either the upper or lower laser
	 *  bank.  The device returns three times as many upper bank blocks.
	 *
	 *  use stdint.h types, so things work with both 64 and 32-bit machines
	 */
	// block
	typedef struct raw_block
	{
		uint16_t header;        ///< UPPER_BANK or LOWER_BANK
		uint8_t rotation_1;
		uint8_t rotation_2;     ///combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
		uint8_t  data[BLOCK_DATA_SIZE]; //96
	} raw_block_t;
	
	/** used for unpacking the first two data bytes in a block
	 *
	 *  They are packed into the actual data stream misaligned.  I doubt
	 *  this works on big endian machines.
	 */
	union two_bytes
	{
		uint16_t uint;
		uint8_t  bytes[2];
	};

	static const int PACKET_SIZE = 1248;
	static const int BLOCKS_PER_PACKET = 12;
	static const int PACKET_STATUS_SIZE = 4;
	static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

	/** \brief Raw Rsldar packet.
	 *
	 *  revolution is described in the device manual as incrementing
	 *    (mod 65536) for each physical turn of the device.  Our device
	 *    seems to alternate between two different values every third
	 *    packet.  One value increases, the other decreases.
	 *
	 *  \todo figure out if revolution is only present for one of the
	 *  two types of status fields
	 *
	 *  status has either a temperature encoding or the microcode level
	 */
	typedef struct raw_packet
	{
		raw_block_t   blocks[BLOCKS_PER_PACKET];
		uint16_t      revolution;
		uint8_t       status[PACKET_STATUS_SIZE];
	} raw_packet_t;
	
	/** \brief RSLIDAR data conversion class */
    class RawData
    {
    public:
        RawData();
        ~RawData() {}
		/*init the size of the scan point size */
        void    init_setup();
        /*load the cablibrated files: angle, distance, intensity*/
        void    loadConfigFile(ros::NodeHandle private_nh);
        /*unpack the UDP packet and opuput PCL PointXYZI type*/
        void    unpack(const rslidar_msgs::rslidarPacket &pkt,pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,bool finish_packets_parse);
        /*calibrated the disctance*/
        float   pixelToDistance(int pixelValue, int passageway);
        /*calibrated the intensity*/
        float   calibrateIntensity(float inten,int calIdx,int distance);

    };


    cv::Mat mat_depth;// = cv::Mat::zeros(16,2100,cv::CV_32F); // 10HZ 2000 points per circle
    //cv::Mat mat_inten;// = cv::Mat::zeros(16,2100,cv::CV_U8C);
    float   VERT_ANGLE[16];
    float   aIntensityCal[1600][16];
    int     g_ChannelNum[16];

    void   removeOutlier(pcl::PointCloud<pcl::PointXYZI>::Ptr);
    
    rslidar_msgs::rslidarPic pic;


} // namespace rslidar_rawdata

#endif // __RAWDATA_H
