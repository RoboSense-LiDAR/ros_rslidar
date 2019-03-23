/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *
 *
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
 *  Input classes for the RSLIDAR RS-16 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __RSLIDAR_INPUT_H_
#define __RSLIDAR_INPUT_H_

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>

namespace rslidar_driver
{
static uint16_t MSOP_DATA_PORT_NUMBER = 6699;   // rslidar default data port on PC
static uint16_t DIFOP_DATA_PORT_NUMBER = 7788;  // rslidar default difop data port on PC
                                                /**
                                                 *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包
                                                 * @brief The Input class,
                                                     *
                                                     * @param private_nh  一个NodeHandled,用于通过节点传递参数
                                                     * @param port
                                                     * @returns 0 if successful,
                                                     *          -1 if end of file
                                                     *          >0 if incomplete packet (is this possible?)
                                                 */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);

  virtual ~Input()
  {
  }

  virtual int getPacket(rslidar_msgs::rslidarPacket* pkt, const double time_offset) = 0;

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
};

/** @brief Live rslidar input from socket. */
class InputSocket : public Input
{
public:
  InputSocket(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER);

  virtual ~InputSocket();

  virtual int getPacket(rslidar_msgs::rslidarPacket* pkt, const double time_offset);

private:
private:
  int sockfd_;
  in_addr devip_;

  int Ret;
  int len;
};

/** @brief rslidar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap
   */
class InputPCAP : public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0,
            std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

  virtual ~InputPCAP();

  virtual int getPacket(rslidar_msgs::rslidarPacket* pkt, const double time_offset);

private:
  ros::Rate packet_rate_;
  std::string filename_;
  pcap_t* pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};
}

#endif  // __RSLIDAR_INPUT_H
