/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *

 Copyright (C) 2010-2013 Austin Robot Technology, and others
 All rights reserved.


Modified BSD License:
--------------------

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the names of the University of Texas at Austin, nor
      Austin Robot Technology, nor the names of other contributors may
      be used to endorse or promote products derived from this
      software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 *
 *  ROS driver interface for the RSLIDAR 3D LIDARs
 */
#ifndef _RSDRIVER_H_
#define _RSDRIVER_H_

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_driver/rslidarNodeConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "input.h"

namespace rslidar_driver
{
class rslidarDriver
{
public:
  /**
 * @brief rslidarDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  rslidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~rslidarDriver()
  {
  }

  bool poll(void);
  void difopPoll(void);

private:
  /// Callback for dynamic reconfigure
  void callback(rslidar_driver::rslidarNodeConfig& config, uint32_t level);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
  } config_;

  boost::shared_ptr<Input> msop_input_;
  boost::shared_ptr<Input> difop_input_;
  ros::Publisher msop_output_;
  ros::Publisher difop_output_;
  // Converter convtor_;
  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  boost::shared_ptr<boost::thread> difop_thread_;
};

}  // namespace rslidar_driver

#endif
