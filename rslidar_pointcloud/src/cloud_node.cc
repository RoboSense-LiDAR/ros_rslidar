/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 Robosense, Tony Zhang
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

    This ROS node converts raw RSLIDAR LIDAR packets to PointCloud2.

*/
#include "convert.h"

/** Main node entry point. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rscloud_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  rslidar_pointcloud::Convert conv(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
