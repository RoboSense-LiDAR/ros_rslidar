#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_pointcloud/CloudNodeConfig.h>
#include "rawdata.h"
namespace rslidar_pointcloud
{
  class Convert
  {
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}

  private:
    
    void callback(rslidar_pointcloud::CloudNodeConfig &config,
                              uint32_t level);
      void processScan( const rslidar_msgs::rslidarScan::ConstPtr &scanMsg);

    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<rslidar_pointcloud::
      CloudNodeConfig> > srv_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud;
    boost::shared_ptr<rslidar_rawdata::RawData> data_;
    ros::Subscriber rslidar_scan_;
    ros::Publisher output_;
    /// configuration parameters
    struct
    {
      std::string frame_id;            ///< tf frame ID
      std::string model;               ///< device model name
      int         npackets;            ///< number of packets to collect
      double      rpm;                 ///< device rotation rate (RPMs)
      double      time_offset;         ///< time in seconds added to each  time stamp
    } config_;
  };

}//namespace velodyne_pointcloud
#endif
