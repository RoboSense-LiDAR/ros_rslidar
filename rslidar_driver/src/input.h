/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2017 robosense technology, guoleiming
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */
/** \file
 *
 *  Input classes for the RSlidar 3D LIDAR:
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
#include <rslidar_msgs/rslidarScan.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>


namespace rs_driver
{
static uint16_t DATA_PORT_NUMBER = 6677;     // rslidar default data port

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
    virtual ~Input() {}
    virtual int getPacket(rslidar_msgs::rslidarPacket *pkt,
                          const double time_offset) = 0;

protected:
    ros::NodeHandle private_nh_;
    uint16_t port_;
    std::string devip_str_;
};

/** @brief Live rslidar input from socket. */
class InputSocket: public Input
{
public:
    InputSocket(ros::NodeHandle private_nh,
                uint16_t port = DATA_PORT_NUMBER);
    virtual ~InputSocket();

    virtual int getPacket(rslidar_msgs::rslidarPacket *pkt,
                          const double time_offset);
    void setDeviceIP( const std::string& ip );
private:

private:
    int sockfd_;
    in_addr devip_;

    char *send_IP;
    int port_dest;
    int port_local;
    int Ret ;
    int len;
    sockaddr_in sender_address;
    socklen_t sender_address_len;
};


/** @brief rslidar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap
   */
class InputPCAP: public Input
{
public:
    InputPCAP(ros::NodeHandle private_nh,
              uint16_t port = DATA_PORT_NUMBER,
              double packet_rate = 0.0,
              std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0);
    virtual ~InputPCAP();

    virtual int getPacket(rslidar_msgs::rslidarPacket *pkt,
                          const double time_offset);
    void setDeviceIP( const std::string& ip );

private:
    ros::Rate packet_rate_;
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
};

}

#endif // __RSLIDAR_INPUT_H
