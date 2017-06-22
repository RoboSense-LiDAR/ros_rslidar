#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
      data_(new rslidar_rawdata::RawData())
  {
    data_->loadConfigFile();   			//load lidar parameters
    data_->init_setup();
    // get model name, validate string, determine packet rate
    private_nh.param("model", config_.model, std::string("RS16")); //lidar model
    double packet_rate;                   // packet frequency (Hz)
    std::string model_full_name;
  if (config_.model == "RS16")
    {
        packet_rate = 834;
        model_full_name = "RS_16";
    }
    else
    {
        ROS_ERROR_STREAM("unknown LIDAR model: " << config_.model);
        packet_rate = 2600.0;
    }
   private_nh.param("rpm", config_.rpm, 600.0);
      //ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
   double frequency = (config_.rpm / 60.0);     // expected Hz rate

   int npackets = (int) ceil(packet_rate / frequency);
   private_nh.param("npackets", config_.npackets, npackets);
    // std::string deviceName(std::string("rslidar ") + model_full_name);
    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<rslidar_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    rslidar_scan_ =
      node.subscribe("rslidar_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
void Convert::callback(rslidar_pointcloud::CloudNodeConfig &config,
                              uint32_t level)
{
    ROS_INFO("Reconfigure Request");
   // config_.time_offset = config.time_offset;
}

  /** @brief Callback for raw scan messages. */
  void Convert::processScan( const rslidar_msgs::rslidarScan::ConstPtr &scanMsg)
  {
  	pcl::PointCloud<pcl::PointXYZI>::Ptr outPoint(new pcl::PointCloud<pcl::PointXYZI>);
  	outPoint->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  	outPoint->header.frame_id = scanMsg->header.frame_id;
    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
       data_->unpack(scanMsg->packets[i], outPoint);
    }
    
    //ROS_DEBUG_STREAM("Publishing height" << outPoint->height<<"width" << outPoint->width
                     //<< " Rslidar points, time: " << outPoint->header.stamp);
    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*outPoint, outMsg);
    output_.publish(outMsg);
  }
} // namespace velodyne_pointcloud
