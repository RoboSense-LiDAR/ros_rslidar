#ifndef _CONVERTION_H_
#define _CONVERTION_H_
#include <ros/ros.h>
#include <ros/package.h>
#include <rslidar/rslidarPic.h>
#include <rslidar/rslidarPacket.h>
#include <rslidar/rslidarScan.h>
#include "myparam.h"
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
/**
 *   convert a rslidarPacket to pcl point cloud
 */
namespace rs_driver
{
	int   a_channelOrder[16] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};
	int    debug_a = 0;
	void    init_setup();
	void    loadConfigFile();
	void    unpack(const rslidar::rslidarPacket &pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr);
	
	float   VERT_ANGLE[16];
	cv::Mat mat_depth;// = cv::Mat::zeros(16,2100,cv::CV_32F); // 10HZ 2000 points per circle
	cv::Mat mat_inten;// = cv::Mat::zeros(16,2100,cv::CV_U8C);

	float   aIntensityCal[1600][16];
    float   inPwrCurveDat[495];
    int     g_ChannelNum[16];
    
    float   pixelToDistance(float pixelValue, int passageway);
    float   calibrateIntensity(float inten,int calIdx,int distance);
    
    void   removeOutlier(pcl::PointCloud<pcl::PointXYZI>::Ptr);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr PCloudRemove(new pcl::PointCloud<pcl::PointXYZI>);
    
    rslidar::rslidarPic pic;


}
#endif
