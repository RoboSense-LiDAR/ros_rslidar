#ifndef LIDAR_ROBOSENSE_H
#define LIDAR_ROBOSENSE_H

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   test1.cpp
 * Author: n
 *
 * a dummy test of rs-lidar reading and pub
 *
 * Created on January 16, 2017, 3:28 PM
 */

#include <cstdlib>
#include <string>
#include <iostream>

#include <fstream>
#include <math.h>

#include <pcap.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;

sensor_msgs::PointCloud2 ros_pcl_msg;

static pcl::PointCloud<pcl::PointXYZ> ros_pcl;

int cloud_write_idx_;

typedef struct TotalPackage {

    unsigned int Azimuth[12][2];
    unsigned char temperature1[12][2];
    unsigned char temperature2[12][2];

    unsigned int Distance[12][32][2];
    unsigned char Atten[12][32];



}TotalPackage;

typedef struct Usefulmessage {
    float JIAODU_[12];
    float JIAODU2_[12];
    double WENDU1_;
    double WENDU2_;
    int JULI_[12][32];
    int PASSEGEWAY_[12][32];
    int PointPos[12][32];
    int intensity[12][32];

}Usefulmessage;

int m_AzimuthValue[12];
int Distance_[12][32];
int Atten_[12][32];
float Distance_Value_[12][32];

double temperature1_;
double temperature2_;
float Azimuth_Value_[12];
//int Distance_[12][32];
int Azimuth_[12];

TotalPackage  *m_Total_Data;
Usefulmessage UsefulData;

int g_ChannelNum[16];

//int g_ChannelNum[16] = {                        //-----------------------------FDQ_change--------------------
//        171, 195, 168, 173,
//        171, 160, 172, 168,
//        172, 168, 180, 190,
//        178, 175, 175, 178};
float PI=3.1415926535897;

/*     
 *
 */
namespace lidar_robosense {
void growpcl(pcl::PointCloud<pcl::PointXYZ> const& cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout)
{
    enum { MAX_CLOUD_SIZE = 60000 };

    size_t new_cloud_size = std::min<size_t>(cloudout.points.size() + cloudin.points.size(), MAX_CLOUD_SIZE);
    cloudout.points.resize(new_cloud_size);

    //std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~the ros_pcl size is ~~~~~~~~~~~~~~~~~~~~~~~~~"<<cloudout.points.size()<<std::endl;
    for (unsigned int i= 0; i < cloudin.points.size(); ++i)
    {
        cloudout.points[cloud_write_idx_].x=cloudin.points[i].x;
        cloudout.points[cloud_write_idx_].y=cloudin.points[i].y;
        cloudout.points[cloud_write_idx_].z=cloudin.points[i].z;

        cloud_write_idx_++;
        if (cloud_write_idx_ == MAX_CLOUD_SIZE) {
            cloud_write_idx_ = 0;
        }
    }

    //cloudout.header=cloudin.header;

    pcl::toROSMsg(cloudout,ros_pcl_msg);
}

void Convert_Azimuth(int Src_Azimuth_[], float Dst_Azimuth_[], int size)
{
    //cout<<"i am in convert azimuth"<<endl;

    for (int i = 0; i < size; i++)
    {

        Dst_Azimuth_[i] = (Src_Azimuth_[i] * 0.1*0.1);
    }
    //cout<<"i am finished convert azimuth"<<endl;
}

int BinTenToSum(unsigned int data[2])
{
    int Sum = 0;
    //printf("data[0] = %d ,data[1] = %d\n", data[0], data[1]);
    Sum = data[0] * 256 + data[1];
    //inttohex[data[0]];
    return Sum;
}

//used
float PixelToDistance(float pixelValue, int passageway, double temperature)
{
    float DistanceValue = 0;
    double  deta;
    //if (pixelValue < 1670*5 || pixelValue > 11670*5)
    if (pixelValue < 200 || pixelValue > 11670)
    {
        DistanceValue = 0;
    }
    else
    {
        //DistanceValue = (((float)pixelValue + channel_));/------------------------------fdq_change
     /*  if (temperature < 30)
        {
            //deta = 30;
            deta = 32;
        }
        else
        {
            switch (int(temperature + 0.5))
            {

            case 30:deta = 32; break;
            case 31:deta = 31; break;
            case 32:deta = 30; break;
            case 33:deta = 29; break;
            case 34:deta = 28; break;
            case 35:deta = 27; break;
            case 36:deta = 26; break;
            case 37:deta = 25; break;
            case 38:deta =24; break;
            case 39:deta = 23; break;
            case 40:deta = 22; break; case 41: deta = 21; break;
            case 42: deta = 20; break; case 43: deta = 19; break;
            case 44:deta = 18; break; case 45: deta = 17; break;
            case 46:deta = 16; break; case 47: deta = 15; break;
            case 48:deta = 14; break; case 49: deta = 13; break;
            case 50:deta = 11; break; case 51: deta = 9; break;
            case 52: deta = 7; break; case 53: deta = 5; break;
            case 54:deta = 3; break; case 55: deta = 1; break;
            case 56:deta = 0; break; case 57: deta = -2; break;
            case 58:deta = -3; break; case 59: deta = -4; break;
            case 60:deta = -6; break; case 61: deta = -7; break;
            case 62:deta = -8; break; case 63: deta = -9; break;
            case 64:deta = -10; break;
            case 65: deta = -11; break;
            case 66:deta = -12; break;
            case 67: deta = -11;  break;
            case 68:deta = -12;  break;
            case 69: deta = -13; break;
            case 70:deta = -14;  break;
            }
        }*/
        deta = 0;
        DistanceValue = (((float)pixelValue - g_ChannelNum[passageway - 1] + deta));

        //temptemperature = temperature;

    }

    return DistanceValue;
}

//pitch angle. replace to LM-5 pitch angle
void SwicthZAizValue(int &count)
{
    if (count == 1){ count =- 15.5386; }
    else if (count == 2){ count = 15.1803; }
    else if (count == 3){ count = -13.4140; }
    else if (count == 4){ count = 13.0557; }
    else if (count == 5){ count = -11.3217; }
    else if (count == 6){ count = 10.9634; }
    else if (count == 7){ count = -9.2580; }
    else if (count == 8){ count = 8.8997; }
    else if (count == 9){ count = -7.2193; }
    else if (count == 10){ count = 6.8611; }
    else if (count == 11){ count = -5.1951; }
    else if (count == 12){ count = 4.8368; }
    else if (count == 13){ count =- 3.1851; }
    else if (count == 14){ count = 2.8304; }
    else if (count == 15){ count = -1.2146; }
    else if (count == 16){ count = 0.8205; }
}


//polar axis to cardianal axis for 1-32 channel
// chazhi
void slotDrawNewData(Usefulmessage Data)
{

    // cout<<"i am in slotDrawNewData"<<endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.points.resize(384);

    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            SwicthZAizValue(Data.PASSEGEWAY_[i][j]);
            //cout<<"Data Passegeway : "<<Data.PASSEGEWAY_[i][j]<<endl;
        }
    }

    ///  cout<<"i am before interplate "<<endl;
    //interplate 16-31 channel
    for (int i = 1; i < 12; i++)
    {
        Data.JIAODU2_[0] = Data.JIAODU_[0] + 0.1;
        if ((Data.JIAODU_[i - 1] - Data.JIAODU_[i])>300)
        {
            Data.JIAODU2_[i] = Data.JIAODU_[i] + 360;
        }
        Data.JIAODU2_[i] = Data.JIAODU_[i] + (Data.JIAODU_[i] - Data.JIAODU_[i - 1]) / 2;
        if (Data.JIAODU2_[i] > 360)
        {
            Data.JIAODU2_[i] = Data.JIAODU2_[i] - 360;
        }
        if (Data.JIAODU2_[i] < 0)
        {
            Data.JIAODU2_[i] = Data.JIAODU2_[i] + 180;
        }

    }

    // cout<<"i am after  interplate "<<endl;

    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j <32; j++)
        {
            // cout<<"so what is the problem here "<<endl;
            /*
            cloud->points[Data.PointPos[i][j]].x = -1 * Data.JULI_[i][j] * cos(Data.JIAODU_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j] / 180.0*PI);		//angle
            cloud->points[Data.PointPos[i][j]].y = Data.JULI_[i][j] * sin(Data.JIAODU_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j] / 180.0*PI);		//distance
            cloud->points[Data.PointPos[i][j]].z = Data.JULI_[i][j] * sin(Data.PASSEGEWAY_[i][j] / 180.0*PI);
            if (j > 15)
            {
                cloud->points[Data.PointPos[i][j]].x = -1 * Data.JULI_[i][j - 16] * cos(Data.JIAODU2_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j - 16] / 180.0*PI);
                cloud->points[Data.PointPos[i][j]].y = Data.JULI_[i][j - 16] * sin(Data.JIAODU2_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j - 16] / 180.0*PI);		//distance
                cloud->points[Data.PointPos[i][j]].z = Data.JULI_[i][j - 16] * sin(Data.PASSEGEWAY_[i][j - 16] / 180.0*PI);
                        }
            */

            //cout <<"Data.JIAODU_[i]:"<< Data.JIAODU_[i]<<endl;
            cloud.points[i*32+j].x = 0.01*(-1 * Data.JULI_[i][j] * cos(Data.JIAODU_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j] / 180.0*PI));		//angle
            cloud.points[i*32+j].y = 0.01*(Data.JULI_[i][j] * sin(Data.JIAODU_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j] / 180.0*PI));		//distance
            cloud.points[i*32+j].z = 0.01*(Data.JULI_[i][j] * sin(Data.PASSEGEWAY_[i][j] / 180.0*PI));
            if (j > 15)
            {
                cloud.points[i*32+j].x = 0.01*(-1 * Data.JULI_[i][j - 16] * cos(Data.JIAODU2_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j - 16] / 180.0*PI));
                cloud.points[i*32+j].y = 0.01*(Data.JULI_[i][j - 16] * sin(Data.JIAODU2_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j - 16] / 180.0*PI));		//distance
                cloud.points[i*32+j].z = 0.01*(Data.JULI_[i][j - 16] * sin(Data.PASSEGEWAY_[i][j - 16] / 180.0*PI));
            }


            //cloud.points[i*32+j].x = -1 * Data.JULI_[i][j] * cos(Data.JIAODU_[i] / 180.0*PI)*cos(Data.PASSEGEWAY_[i][j] / 180.0*PI);

            //cout <<"cloud.points[i*32+j].x :"<< cloud.points[i*32+j].x<<endl;
            //cout <<"cloud.points[i*32+j].y :"<< cloud.points[i*32+j].y<<endl;
            //cout <<"cloud.points[i*32+j].z :"<< cloud.points[i*32+j].z<<endl;

            //cout<<"can it be point cloud pass ? "<<endl;
            //Vec.push_back(ThreeDcloud_->points[Data.PointPos[i][j]]);

        }
    }

    //cout<<"i am finish interplate "<<endl;
    //cloud.width = cloud.size();	//poitcloud size
    //cloud.height = 1;
    //cloud.header.frame_id = "velodyne";

    // pub msg here
    //cloud.header.stamp = ros::Time::now();
    //cloud.header.frame_id = "velodyne";


    // pcl::toROSMsg(cloud, ros_pcl_msg);

    growpcl(cloud,ros_pcl);
    // cout<<"finish rosmsg "<<endl;

}

void UDP_Parse(const u_char *data)
{
    // cout <<"data + 184 : " <<endl;

    //printf("%d ", *(data+184));

    //cout <<"data + 185  : " <<endl;

    // printf("%d ", *(data+185));

    temperature1_ = *(data+181) + *(data+182)*0.0625;
    temperature2_ = *(data+183) + *(data+184)*0.0625; ///problem

    for (int i = 0; i < 12; i++)
    {
        //cout << " i am before passing data"<<endl;
        //correct angle

        //0xffee
        if(*(data + 184 + 100 * i)== 255 && *(data + 184+ 1 + 100 * i)== 238)
        {
            //cout << " i am correct "<<endl;
            // 计算角度
            int test1 = *(data + 184 + 2 + 100 * i);
            int test2 = *(data + 184 + 3 + 100 * i);
            //cout << " a test "<<endl;

            //m_Total_Data->Azimuth[i][0] = *(data + 42 + 2 + 100 * i);	//255	ee = 238
            //m_Total_Data->Azimuth[i][1] = *(data + 42 + 3 + 100 * i);
            //printf("%d ", *(data + 42 + 2 + 100 * i));
            //cout << " i am after passing data"<<endl;
            //Azimuth_[i] = BinTenToSum(m_Total_Data->Azimuth[i]); //int Azimuth_[12];
            Azimuth_[i]= test1*256+test2;
        }

        else
        {
            // cout << " error match"<<endl;
        }
        // cout<<"i am in loop"<<endl;
    }

    //cout<<"i am out of  loop"<<endl;

    Convert_Azimuth(Azimuth_, Azimuth_Value_, 12);
    //-----------------------------------------------solve distance and luminous attenuation ---------------------------------------

    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 32; j++)
        {
            //m_Total_Data->Distance[i][j][0] = *(data + 42 + 4 + 3 * j + 100 * i);
            //m_Total_Data->Distance[i][j][1] = *(data + 42 + 5 + 3 * j + 100 * i);
            //m_Total_Data->Atten[i][j] = *(data + 42 + 6 + 3 * j + 100 * i);
            int test_dist_1 = *(data + 184 + 4 + 3 * j + 100 * i);
            int test_dist_2 = *(data + 184 + 5 + 3 * j + 100 * i);
            int test_atten = *(data + 184 + 6 + 3 * j + 100 * i);

            Distance_[i][j] = test_dist_1*256+test_dist_2;
            //ROS_INFO("distance: %d", Distance_[i][j]);
            //Distance_[i][1] = Distance_[i][3];
            //Distance_[i][j] = BinTenToSum(m_Total_Data->Distance[i][j]);
            //cout<<"i before PixelToDistance  loop"<<endl;

            Distance_Value_[i][j] = Distance_[i][j];//PixelToDistance(Distance_[i][j], j + 1, temperature1_);
            //ROS_INFO("distance2: %f", Distance_Value_[i][j]);

            //cout<<"i after PixelToDistance  loop"<<endl;

            Atten_[i][j] =  test_atten;
            //Atten_[i][j] = m_Total_Data->Atten[i][j];
        }

    }
    //----------------------------useful data------------------------------------
    //Azimuth_Value_	Distance_Value_

    //Usefulmessage UsefulData;

    //cout<<"i before memset"<<endl;

    memset(&UsefulData, 0, sizeof(Usefulmessage));

    //cout<<"i after memset"<<endl;

    for (int i = 0; i < 12; i++)
    {
        UsefulData.JIAODU_[i] = Azimuth_Value_[i];
    }

    UsefulData.WENDU1_ = temperature1_;
    UsefulData.WENDU2_ = temperature2_;
    for (int j = 0; j < 12; j++)
    {
        for (int i = 0; i < 32; i++)
        {
            UsefulData.JULI_[j][i] = Distance_Value_[j][i];
            UsefulData.PASSEGEWAY_[j][i] = i + 1;
            UsefulData.PointPos[j][i] = i + 32 * j + 32 * 12;
            UsefulData.intensity[j][i] = Atten_[j][i];
        }
    }

    //cout<<"i before slotDrawNewData"<<endl;

    slotDrawNewData(UsefulData);

    //memset(m_Total_Data, 0, sizeof(m_Total_Data));


}


/*
int main(int argc, char** argv) {


    ros::init(argc, argv, "RS_Lidar");

    ros::NodeHandle n;

    ros::Publisher point_cloud_pub_;

    ros::Rate rate(1000);


    point_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 60000);

    string file = "/home/caishuai/data/rslidar.pcap";

    char errbuff[PCAP_ERRBUF_SIZE];

    pcap_t * pcap = pcap_open_offline(file.c_str(), errbuff);
    struct pcap_pkthdr *header;
    const u_char *data;

    u_int packetCount = 0;




    while (ros::ok())
    {

        int returnValue = pcap_next_ex(pcap, &header, &data);

        //printf("Packet # %i\n", ++packetCount);

        UDP_Parse(data);

        //cout<<"i am in ros loop ..."<<endl;

        ros_pcl_msg.header.stamp = ros::Time::now();

        ros_pcl_msg.header.frame_id = "velodyne";



        point_cloud_pub_.publish (ros_pcl_msg);

        ros::spinOnce ();

        rate.sleep ();
    }

    // Show Epoch Time
    //printf("Epoch Time: %ld:%ld seconds\n", header->ts.tv_sec, header->ts.tv_usec);

    // loop through the packet and print it as hexidecimal representations of octets
    // We also have a function that does this similarly below: PrintData()


    // Add two lines between packets
    // printf("\n\n");


    // allocate an output point cloud with same time as raw data

    return 0;
}

*/

}

#endif // LIDAR_ROBOSENSE_H
