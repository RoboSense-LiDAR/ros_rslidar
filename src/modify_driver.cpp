
#include <string>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rslidar/rslidarScan.h>
#include "input.h"
#include "modify_driver.h"

namespace rslidar_driver
{

rslidarDriver::rslidarDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
    // use private node handle to get parameters
    private_nh.param("frame_id", config_.frame_id, std::string("rslidar"));
    std::string tf_prefix = tf::getPrefixParam(private_nh);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

    // get model name, validate string, determine packet rate
    private_nh.param("model", config_.model, std::string("VLP16"));
    double packet_rate;                   // packet frequency (Hz)
    std::string model_full_name;
    if ((config_.model == "64E_S2") ||
            (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
        packet_rate = 3472.17;            // 1333312 / 384
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "64E")
    {
        packet_rate = 2600.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32E")
    {
        packet_rate = 1808.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "VLP16")
    {
        packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
        model_full_name = "VLP-16";
    }
    else
    {
        ROS_ERROR_STREAM("unknown LIDAR model: " << config_.model);
        packet_rate = 2600.0;
    }
    std::string deviceName(std::string("rslidar ") + model_full_name);

    private_nh.param("rpm", config_.rpm, 600.0);
    //ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0);     // expected Hz rate

    // default number of packets for each scan is a single revolution
    // (fractions rounded up)
    config_.npackets = (int) ceil(packet_rate / frequency);
    private_nh.getParam("npackets", config_.npackets);
    //ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

    std::string dump_file;
    private_nh.param("pcap", dump_file, std::string(""));

    int udp_port;
    private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

    // Initialize dynamic reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<rslidar::
            rslidarNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<rslidar::rslidarNodeConfig>::
            CallbackType f;
    f = boost::bind (&rslidarDriver::callback, this, _1, _2);
    srv_->setCallback (f); // Set callback function und call initially

    // initialize diagnostics
    diagnostics_.setHardwareID(deviceName);
    const double diag_freq = packet_rate/config_.npackets;
    diag_max_freq_ = diag_freq;
    diag_min_freq_ = diag_freq;
    //ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("rslidar_packets", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                               &diag_max_freq_,
                                                               0.1, 10),
                                          TimeStampStatusParam()));

    // open rslidar input device or file
    if (dump_file != "")                  // have PCAP file?
    {
        // read data from packet capture file
        input_.reset(new rslidar_driver::InputPCAP(private_nh, udp_port,
                                                    packet_rate, dump_file));
    }
    else
    {
        // read data from live socket
        input_.reset(new rslidar_driver::InputSocket(private_nh, udp_port));
    }

    // raw packet output topic
    output_ =
            node.advertise<rslidar::rslidarScan>("rslidar_packets", 10);

    /// 读参数文件 2017-02-27
    FILE *inIntenCal = fopen("./src/rslidar/data/curves.csv", "r");   // path
    FILE *inPwrCurve = fopen("./src/rslidar/data/pwrCurves.csv","r");
     FILE *angleID = fopen("./src/rslidar/data/angle.csv","r");
     FILE *channel=fopen("./src/rslidar/data/ChannelNum.csv","r");
    int loopi=0;
    int loopj = 0;
    while (~feof(inIntenCal)) {
        float a[16];
        loopi++;
        if (loopi > 1600)
            break;
        fscanf(inIntenCal, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
               &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15]);
        for (loopj = 0; loopj < 16; loopj++){
            inIntenCalDat[loopi - 1][loopj] = a[loopj];

            //ROS_INFO("in: %f", inIntenCalDat[loopi - 1][loopj]);
        }
    }

    for (loopi = 0; loopi <495; loopi++){
        fscanf(inPwrCurve, "%f\n", &inPwrCurveDat[loopi]);
        //ROS_INFO("in2: %f", inPwrCurveDat[loopi]);
        //fprintf(outCalVfy, "%d\n", inPwrCurveDat[loopi] );
    }
    /*********read veat_angle data******/
    float b[16];
    int loopk=0;
    int loopn=0;
    while(~feof(angleID))
    {
        fscanf(angleID,"%f",&b[loopk]);
        loopk++;
        if(loopk>15) break;
    }
    for(loopn=0;loopn<16;loopn++)
    {
        VERT_ANGLE[loopn]=b[loopn]/180*3.1415926;
    }
    /*********read veat_angle data******/
    int loopl=0;
  //  int c[16];
    while(~feof(channel))
    {
        fscanf(channel,"%d",&g_ChannelNum[loopl]);
       // ln[loopl]=g_ChannelNum[loopl];
      //  ROS_INFO(" %d",g_ChannelNum[loopk]);
        loopl++;
        if(loopl>15) break;
    }
    /**********************************/
    ///End of Initial for the Calibration ROM
    fclose(inIntenCal);
    fclose(inPwrCurve);
    fclose(angleID);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool rslidarDriver::poll(void)
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    rslidar::rslidarScanPtr scan(new rslidar::rslidarScan);
    scan->packets.resize(config_.npackets);

    // Since the rslidar delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    for (int i = 0; i < config_.npackets; ++i)
    {
        while (true)
        {
            // keep reading until full packet received
            int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
            if (rc == 0) break;       // got a full packet?
            if (rc < 0) return false; // end of file reached?
        }
    }
    //ROS_INFO("poll2");
    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full rslidar scan.");
    scan->header.stamp = scan->packets[config_.npackets - 1].stamp;
    scan->header.frame_id = config_.frame_id;
    output_.publish(scan);
    for(size_t i=0; i<scan->packets.size(); ++i)
    {
        unpack(scan->packets[i]);
    }

    // notify diagnostics that a message has been published, updating
    // its status
    //diag_topic_->tick(scan->header.stamp);
    //diagnostics_.update();

    return true;
}

void rslidarDriver::callback(rslidar::rslidarNodeConfig &config,
                              uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    config_.time_offset = config.time_offset;
}

}
