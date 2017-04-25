#ifndef CONVERT_FULLSCAN_
#define CONVERT_FULLSCAN_

#include <ros/ros.h>
#include <rslidar/rslidarPic.h>
#include <rslidar/rslidarPacket.h>
#include <rslidar/rslidarScan.h>
#include "myparam.h"
#include "lidar_robosense.h"
int fullscan = 0;
ros::Publisher pic_output;
ros::Publisher pc_output;
rslidar::rslidarPic pic;
//int correct[16] = {0, 2, 4, 6, 8, 10, 12, 14, 15, 13, 11, 9, 7, 5, 3, 1};
int correct[16] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};
const float VERT_ANGLE[] = {
  -14.9843/180*3.1415926,      //-15
  -13.0293/180*3.1415926,    //-13
  -10.9738/180*3.1415926,    //-11
  -9.0641/180*3.1415926,    //-9
  -6.9769/180*3.1415926,    //-7
  -5.0451/180*3.1415926,    //-5
   -2.9785/180*3.1415926,    //-3
   -1.0115/180*3.1415926,  //-1
     0.9936/180*3.1415926,   //1
     2.9606/180*3.1415926,     //3
     4.9918/180*3.1415926,     //5
     7.0104/180*3.1415926,     //7
     8.9768/180*3.1415926,     //9
    10.9220/180*3.1415926,     //11
    13.0123/180*3.1415926,     //13
    15.0345/180*3.1415926  //15//LM26
};


//光衰信息表
float inIntenCalDat[1600][16];
//充能时间-实际功率对应表
float  inPwrCurveDat[495];
 
void init_setup()
{
    pic.col=0;
    pic.distancenum=0;
    pic.intensitynum=0;
    pic.azimuth.resize(POINT_PER_CIRCLE_);
    pic.distance.resize(DATA_NUMBER_PER_SCAN);
    pic.intensity.resize(DATA_NUMBER_PER_SCAN);
    pic.azimuthforeachP.resize(DATA_NUMBER_PER_SCAN);
}

void unpack(const rslidar::rslidarPacket &pkt)
{
    float azimuth;
    float intensity;
    float azimuth_diff;
    float last_azimuth_diff;
    float azimuth_corrected_f;
    int azimuth_corrected;

    /// 解析温度值
    //float temperature = pkt.data[38] + pkt.data[39]*0.0625;
    //float temperature2 = pkt.data[40] + pkt.data[41]*0.0625;
    float temperature1_ = 0;
    float temperature2_ = 0;
    float bitneg1 = pkt.data[39] & 128;// 10000000;
    float highbit1 = pkt.data[39] & 127;// 01111111;
    float lowbit1 = pkt.data[38] >> 3;    
    float bitneg2 = pkt.data[41] & 128;
    float highbit2 = pkt.data[41] & 127;
    float lowbit2 = pkt.data[40] >> 3;
    if (bitneg1 == 128)
        temperature1_ = -1 * (highbit1 * 32 + lowbit1)*0.0625;
    else
        temperature1_ = (highbit1 * 32 + lowbit1)*0.0625;
    if (bitneg2 == 128)
        temperature2_ = -1 * (highbit2 * 32 + lowbit2)*0.0625;
    else
        temperature2_ = (highbit2 * 32 + lowbit2)*0.0625;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[42];

    for (int block = 0; block < BLOCKS_PER_PACKET; block++ , ++pic.col)
    {
        if(UPPER_BANK != raw->blocks[block].header)
        {
            ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                     << block << " header value is "
                                     << raw->blocks[block].header);
            return ;
        }

        azimuth = (float)(256*raw->blocks[block].rotation_1+raw->blocks[block].rotation_2)*1;
	int tempAzimuth;
	tempAzimuth = int(azimuth);
	tempAzimuth = (36000+18000 - int(azimuth + 13000)%36000); 

	tempAzimuth = (tempAzimuth + 18000)%36000;

	azimuth = tempAzimuth;

        if (block < (BLOCKS_PER_PACKET-1))//12
        {
            int azi1, azi2;
            azi1 = (float)(256*raw->blocks[block+1].rotation_1+raw->blocks[block+1].rotation_2)*1;
            azi2 = (float)(256*raw->blocks[block].rotation_1+raw->blocks[block].rotation_2)*1;
            azimuth_diff = (float)((36000 + azi1 - azi2)%36000);
            last_azimuth_diff = azimuth_diff;
        }else
        {
            azimuth_diff = last_azimuth_diff;
        }

        if(((pic.col>100)&&(abs(azimuth-pic.azimuth[0])<100))||(pic.col==(POINT_PER_CIRCLE_-2)))
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
            pointcloud->height = 1;
            pointcloud->header.frame_id = "rslidar";
            for (int block = 0; block < pic.col; block++)
            {
                for (int firing = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
                {
                    for (int laserid = 0; laserid < VLP16_SCANS_PER_FIRING; laserid++)
                    {
                        double dis = pic.distance[block*32 + correct[laserid] + 16*firing];
                        double arg_horiz = PI - pic.azimuthforeachP[block*32 + correct[laserid] + 16*firing] /18000*PI;
                        double intensity = pic.intensity[block*32 + correct[laserid] + 16*firing];
                        double arg_vert = VERT_ANGLE[laserid];

                        pcl::PointXYZI point;
                        point.x = dis * cos(arg_vert) * sin(arg_horiz);
                        point.y = dis * cos(arg_vert) * cos(arg_horiz);
                        point.z = dis * sin(arg_vert);
                        point.intensity = intensity;
                        //ROS_INFO("inten: %f", intensity);
                        pointcloud->points.push_back(point);
                    }
                }
            }
            pointcloud->width = pointcloud->points.size();
            pointcloud->resize(pointcloud->height * pointcloud->width);
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*pointcloud, out);
            pc_output.publish(out);

            pic_output.publish(pic);
            init_setup();
            pic.header.stamp = pkt.stamp;
            fullscan = 1;
        }
        pic.azimuth[pic.col]=azimuth;

        for(int firing = 0 ,k = 0;firing < VLP16_FIRINGS_PER_BLOCK; firing++)//2
        {
            for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE)//16   3
            {
                int calIdx = 0;
                int algDist = 0;
                int sDist = 0;
                float  realPwr = 0;
                float refPwr = 0;
                float tempInten = 0;
                float iInten = 0;
                int showPwrs =0 ;
                int showInten = 0;

                azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);// 2.304f    55.296f   110.592f
                azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;//convert to integral value...
                pic.azimuthforeachP[pic.col*32+k/3]=azimuth_corrected;

                union two_bytes tmp;
                tmp.bytes[1] = raw->blocks[block].data[k];
                tmp.bytes[0] = raw->blocks[block].data[k+1];
                float distance = tmp.uint;// * DISTANCE_RESOLUTION;
                calIdx = dsr;
                // 读强度数据
                intensity = raw->blocks[block].data[k+2];
//-------------------------------------------------------------------------------------------------
		//limt intensity data and make it doubled(in FPGA this data is divided by 2)
             /*   intensity = (intensity>3) ? intensity * 2 : 6;
                realPwr = intensity;*/
//-------------------------------------------------------------------------------------------------
                //tempInten=intensity ;
                realPwr= intensity;
                if(int(realPwr)<126)
                	realPwr=realPwr*4;
                else
                	realPwr=(realPwr-125)*16+500;
                
//-------------------------------------------------------------------------------------------------
		//limit sDist belongs to [200,1600] in unit cm
                sDist = (int(distance) > 200) ? int(distance) : 201;
                sDist = (sDist < 1800) ? sDist : 1800;		
		//minus the static offset (this data is For the intensity cal useage only)	
                algDist = int(sDist - 200) ;
                algDist = algDist < 1400? algDist : 1399;
                refPwr = inIntenCalDat[algDist][calIdx];
               //refPwr = refPwr > 6 ? refPwr : 6	; 
               //refPwr=inPerCurveDat[int(refPwr)-6];
               //realPwr=inPerCurveDat[int(realPwr)-6];
                tempInten = refPwr / float(realPwr)	;
                tempInten = tempInten * 200		;  

                tempInten=tempInten>255?255:tempInten;
                iInten = float(tempInten);

                float distance2 = lidar_robosense::PixelToDistance(distance, dsr+1, temperature1_);
                distance2 = 0.01* distance2;

                pic.distance[pic.col*32+k/3] = distance2;
                pic.distancenum++;
                pic.intensity[pic.col*32+k/3] = iInten;
                pic.intensitynum++;

            }
        }

    }
}


#endif


