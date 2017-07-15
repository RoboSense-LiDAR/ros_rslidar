/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  RSLIDAR 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw RSLIDAR LIDAR packets into useful
 *  formats.
 *
 */
#include "rawdata.h"
namespace rslidar_rawdata
{

RawData::RawData() {}
void RawData::loadConfigFile(ros::NodeHandle private_nh)
{
    std::string curvesPath;
    std::string anglePath;
    std::string channelPath;
    std::string pkgPath = ros::package::getPath("rslidar_pointcloud");
    
    private_nh.param("curves_path", curvesPath, std::string(""));
    private_nh.param("angle_path", anglePath,std::string(""));
    private_nh.param("channel_path", channelPath,std::string(""));
    
    curvesPath = pkgPath + curvesPath;
    anglePath = pkgPath + anglePath;
    channelPath = pkgPath + channelPath;
    
    /// 读参数文件 2017-02-27
    FILE *f_inten = fopen(curvesPath.c_str(), "r");   
    int loopi=0;
    int loopj = 0;

    if(!f_inten)
    {
        ROS_ERROR_STREAM(curvesPath <<" does not exist");
    }
    else
    {
        while (~feof(f_inten))
        {
            float a[16];
            loopi++;
            if (loopi > 1600)
                break;
	    fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
               &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15]);
            for (loopj = 0; loopj < 16; loopj++){
                aIntensityCal[loopi - 1][loopj] = a[loopj];
            }
	}
        fclose(f_inten);
     }
    //=============================================================
    FILE *f_angle = fopen(anglePath.c_str(),"r");
    if(!f_angle)
    {
        ROS_ERROR_STREAM(anglePath <<" does not exist");

    }
    else
    {
        float b[16];
        int loopk=0;
        int loopn=0;
        while(~feof(f_angle))
        {
            fscanf(f_angle,"%f",&b[loopk]);
            loopk++;
            if(loopk>15) break;
        }
        for(loopn=0;loopn<16;loopn++)
        {
            VERT_ANGLE[loopn]=b[loopn]/180 * CV_PI;
        }
        fclose(f_angle);
    }

    //=============================================================
    FILE *f_channel = fopen(channelPath.c_str(),"r");
    if(!f_channel)
    {
        ROS_ERROR_STREAM(channelPath <<" does not exist");
    }
    else
    {
        int loopl=0;
        //  int c[16];
        while(~feof(f_channel))
        {
            fscanf(f_channel,"%d",&g_ChannelNum[loopl]);
            loopl++;
            if(loopl>15) break;
        }
        fclose(f_channel);
    }
}
  /** Set up for on-line operation. */
void RawData::init_setup()
{
        pic.col=0;
        pic.azimuth.resize(POINT_PER_CIRCLE_);
        pic.distance.resize(DATA_NUMBER_PER_SCAN);
        pic.intensity.resize(DATA_NUMBER_PER_SCAN);
        pic.azimuthforeachP.resize(DATA_NUMBER_PER_SCAN);
  }
  
float RawData::pixelToDistance(int pixelValue, int passageway)
{
    float DistanceValue;
    
    if(pixelValue <= g_ChannelNum[passageway])
    {
    	DistanceValue = 0.0;
    }
    else
    {
    	DistanceValue = (float)(pixelValue - g_ChannelNum[passageway]);
    }

    
    return DistanceValue;
}

//校准反射强度值
float RawData::calibrateIntensity(float intensity,int calIdx, int distance)
{
	int algDist;
    int sDist;
    int uplimitDist;
    float realPwr;
    float refPwr;
    float tempInten;
	
	uplimitDist = g_ChannelNum[calIdx] + 1400;
	realPwr = intensity;
	
    if((int)realPwr<126)
        realPwr = realPwr * 4.0;
    else
        realPwr = (realPwr-125.0) * 16.0 + 500.0;

    //-------------------------------------------------------------------------------------------------
    //limit sDist belongs to [200,1600] in unit cm
    sDist = (distance > g_ChannelNum[calIdx]) ? distance : g_ChannelNum[calIdx];
    sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
    //minus the static offset (this data is For the intensity cal useage only)
    algDist = sDist - g_ChannelNum[calIdx] ;
    //algDist = algDist < 1400? algDist : 1399;
    refPwr = aIntensityCal[algDist][calIdx];
    tempInten = refPwr / realPwr;
    tempInten = tempInten * 200.0;
    tempInten = (int)tempInten > 255 ? 255.0 : tempInten;
    return tempInten;
    //------------------------------------------------------------
}


  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
 void RawData::unpack(const rslidar_msgs::rslidarPacket &pkt,pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,bool finish_packets_parse)
 {
    float azimuth;  //0.01 dgree
    float intensity;
    float azimuth_diff;
    float last_azimuth_diff;
    float azimuth_corrected_f;
    int   azimuth_corrected;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[42];

    for (int block = 0; block < BLOCKS_PER_PACKET; block++) //1 packet:12 data blocks
    {

        if(UPPER_BANK != raw->blocks[block].header)
        {
            ROS_WARN_STREAM_THROTTLE(60, "skipping invalid RSLIDAR packet: block "
                                     << block << " header value is "
                                     << raw->blocks[block].header);
            
            break ;
        }
        azimuth = (float)(256*raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);
   
        if (block < (BLOCKS_PER_PACKET-1))//12
        {
            int azi1, azi2;
	        azi1 = 256*raw->blocks[block+1].rotation_1 + raw->blocks[block+1].rotation_2;
            azi2 = 256*raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
            azimuth_diff = (float)((36000 + azi1 - azi2)%36000);
             //Debug start by Tony 20170523
            if(azimuth_diff <= 0.0 || azimuth_diff > 70.0){
                //ROS_INFO("Error: %d  %d", azi1, azi2);
                azimuth_diff = 40.0;
                azimuth = pic.azimuth[pic.col-1] + azimuth_diff;
            }
            //Debug end by Tony 20170523
            last_azimuth_diff = azimuth_diff;
        }else
        {
            azimuth_diff = last_azimuth_diff;
        }

        for(int firing = 0 ,k = 0;firing < RS16_FIRINGS_PER_BLOCK; firing++)//2
        {
            for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)//16   3
            {
                int point_count = pic.col * SCANS_PER_BLOCK + dsr + RS16_SCANS_PER_FIRING * firing;
                azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) / RS16_BLOCK_TDURATION);
                azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;//convert to integral value...
                pic.azimuthforeachP[point_count] = azimuth_corrected;

                union two_bytes tmp;
                tmp.bytes[1] = raw->blocks[block].data[k];
                tmp.bytes[0] = raw->blocks[block].data[k+1];
                int distance = tmp.uint;

				// read intensity
                intensity = raw->blocks[block].data[k+2];
                intensity = calibrateIntensity(intensity,dsr,distance);
                
                float distance2 = pixelToDistance(distance, dsr);
                distance2 = distance2 * DISTANCE_RESOLUTION;

                pic.distance[point_count] = distance2;
                pic.intensity[point_count] = intensity;

               }
        }
        pic.azimuth[pic.col] = azimuth;
        pic.col++;
    }

    if(finish_packets_parse)
    {
      //ROS_INFO_STREAM("***************: "<<pic.col);
      pointcloud->clear();
      pointcloud->height = RS16_SCANS_PER_FIRING;
      pointcloud->width = 2 * pic.col;
      pointcloud->is_dense = false;
      pointcloud->resize(pointcloud->height * pointcloud->width);
      mat_depth = cv::Mat(cv::Size( 2 * pic.col,RS16_SCANS_PER_FIRING), CV_32FC1, cv::Scalar(0));
      //mat_inten = cv::Mat(cv::Size(2 * pic.col ,RS16_SCANS_PER_FIRING ), CV_8UC1, cv::Scalar(0));
      for (int block_num = 0; block_num < pic.col; block_num++)
      {

          for (int firing = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++)
          {
              for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++)
              {
                  int point_count = block_num * SCANS_PER_BLOCK + dsr + RS16_SCANS_PER_FIRING * firing;
                  float dis = pic.distance[point_count];
                  float arg_horiz = pic.azimuthforeachP[point_count]/18000*CV_PI;
                  float intensity = pic.intensity[point_count];
                  float arg_vert = VERT_ANGLE[dsr];
                  pcl::PointXYZI point;
                  if(dis > DISTANCE_MAX || dis < DISTANCE_MIN)  //invalid data
                  {
                      point.x = NAN;
                      point.y = NAN;
                      point.z = NAN;
                      point.intensity = 0;
                      //ROS_INFO("inten: %f", intensity);
                      //pointcloud->push_back(point);
                      pointcloud->at(2*block_num + firing, dsr) = point;
                      mat_depth.at<float>(dsr,2*block_num + firing) = 0;
                      //mat_inten.at<uchar>(dsr,2*block_num + firing) = (uchar)0;
                  }else
                  {
                      point.x = dis * cos(arg_vert) * sin(arg_horiz);
                      point.y = dis * cos(arg_vert) * cos(arg_horiz);
                      point.z = dis * sin(arg_vert);
                      point.intensity = intensity;
                      pointcloud->at(2*block_num + firing, dsr) = point;
                      mat_depth.at<float>(dsr,2*block_num + firing) = dis;
                      //mat_inten.at<uchar>(dsr,2*block_num + firing) = (uchar)intensity;

                  }

              }
          }
      }
      removeOutlier(pointcloud); //去除脏数据
      init_setup();
      pic.header.stamp = pkt.stamp;
    }

}


    /***************************************************************************************
    Function:  
    Input:     src 待处理原图像 pt 初始生长点 th 生长的阈值条件
    Output:    肺实质的所在的区域 实质区是白色，其他区域是黑色
    Description: 生长结果区域标记为白色(255),背景色为黑色(0)
    Return:    Mat
    Others:    NULL
    ***************************************************************************************/
   void  regionGrow(cv::Mat src,cv::Mat &matDst, cv::Point2i pt, float th)
    {

        cv::Point2i ptGrowing;                      //待生长点位置
        int nGrowLable = 0;                             //标记是否生长过
        float nSrcValue = 0;                              //生长起点灰度值
        float nCurValue = 0;                              //当前生长点灰度值

        //生长方向顺序数据
       // int DIR[28][2] = {{-1,0}, {1,0}, {-2,0}, {2,0}, {-3,0}, {3,0}, {-4,0}, {4,0},{-5,0}, {5,0} ,{-6,0}, {6,0} ,{-7,0}, {7,0} ,{-8,0}, {8,0},{-9,0}, {9,0},{-10,0}, {10,0},{-11,0}, {11,0},{0,1}, {0,-1},{-1,1}, {-1,-1},{1,1}, {1,-1}};  //由近到远进行搜索
        int DIR[10][2] = {{-1,0}, {1,0}, {-2,0}, {2,0}, {-3,0}, {3,0}, {-4,0}, {4,0}, {0,1}, {0,-1}};  //由近到远进行搜索


        nSrcValue = src.at<float>(pt);            //记录生长点的灰度值
        matDst.at<uchar>(pt)=0;
        if (nSrcValue == 0)
        {
             matDst.at<uchar>(pt)=255;
            return;
        }

            //分别对八个方向上的点进行生长
            for (int i = 0; i<10; ++i)
            {
                ptGrowing.x = pt.x + DIR[i][0];
                ptGrowing.y = pt.y + DIR[i][1];
                //检查是否是边缘点
                if (ptGrowing.x < 0 || ptGrowing.y < 0 || ptGrowing.x > (src.cols-1) || (ptGrowing.y > src.rows -1))
                    continue;


                    nCurValue = src.at<float>(ptGrowing);
/*
                    if (abs(nSrcValue - nCurValue) < th)
                     {
                        matDst.at<uchar>(pt)=255;
                        return;
                     }

*/
                    if(nCurValue == 0)
                    {
                        continue;
                    }

                    if(nSrcValue <= 2  )//在阈值范围内则生长
                       {
                         if (abs(nSrcValue - nCurValue) < 0.2)
                          {
                             matDst.at<uchar>(pt) += 1;
                          }

                          if(matDst.at<uchar>(pt)>=3)           //
                           {
                               matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
                               return;
                           }
                       }
                    if(nSrcValue > 2 && nSrcValue <= 5   )
                      {
                        if (abs(nSrcValue - nCurValue) < 0.5)
                         {
                            matDst.at<uchar>(pt) += 1;
                         }

                         if(matDst.at<uchar>(pt)>=2)           //
                          {
                              matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
                              return;
                          }
                       }
                    if(nSrcValue >5)
                       {

                        if (abs(nSrcValue - nCurValue) < 1)
                         {
                            matDst.at<uchar>(pt) += 1;
                         }

                         if(matDst.at<uchar>(pt)>=1)           //
                          {
                              matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
                              return;
                          }
                       }



           }

      }




void removeOutlier(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud )
{

   cv::Mat matMask = cv::Mat::zeros(mat_depth.size(), CV_8UC1);   //创建一个空白区域，填充为黑色
   //int removeNum =0 , nannum = 0;
   for(int row = 0; row < mat_depth.rows; row++)
     {
        for(int col = 0 ; col < mat_depth.cols ; col++ )
       {
          float d = mat_depth.at<float>(row,col);
          if( d > 8)
          {
              matMask.at<uchar>(row,col) = (uchar)255;
              continue;
          }
          //if(d == 0)
          //{
          //    nannum++;
          //}
          cv::Point2i pt(col,row);
          regionGrow(mat_depth,matMask,pt,1);
       }
    }
   //ROS_INFO("NAN: %d", nannum);
    for(int row = 0; row < matMask.rows; row++)
     {
        for(int col = 0 ; col < matMask.cols ; col++ )
        {
            if(matMask.at<uchar>(row,col) != 255)
            {
               //removeNum++;
               pointcloud->at(col,row).x =NAN;
               pointcloud->at(col,row).y =NAN;
               pointcloud->at(col,row).z =NAN;
               //pointcloud->at(col,row).intensity= 255;
            }

        }
      }

}
}//namespace rs_pointcloud


