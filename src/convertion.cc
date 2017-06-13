#include "convertion.h"
namespace rs_driver
{


void loadConfigFile()
{
    std::string pkgPath = ros::package::getPath("rslidar");
    std::string filePath = pkgPath +  "/data/curves.csv";
    FILE *f_inten = fopen(filePath.c_str(), "r");   // path
    int loopi=0;
    int loopj = 0;

    if(!f_inten)
    {
        ROS_ERROR_STREAM(filePath <<" does not exist");
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
    filePath = pkgPath +  "/data/pwrCurves.csv";
    FILE *f_power = fopen(filePath.c_str(),"r");
    if(!f_power)
    {
        ROS_ERROR_STREAM(filePath <<" does not exist");

    }
    else
    {
        for (loopi = 0; loopi <495; loopi++)
        {
            fscanf(f_power, "%f\n", &inPwrCurveDat[loopi]);
        }

        fclose(f_power);
    }
    //=============================================================
    filePath = pkgPath +  "/data/angle.csv";
    FILE *f_angel = fopen(filePath.c_str(),"r");
    if(!f_angel)
    {
        ROS_ERROR_STREAM(filePath <<" does not exist");

    }
    else
    {
        float b[16];
        int loopk=0;
        int loopn=0;
        while(~feof(f_angel))
        {
            fscanf(f_angel,"%f",&b[loopk]);
            loopk++;
            if(loopk>15) break;
        }
        for(loopn=0;loopn<16;loopn++)
        {
            VERT_ANGLE[loopn]=b[loopn]/180 * 3.1415926;
        }
        fclose(f_angel);
    }

    //=============================================================
    filePath = pkgPath +  "/data/ChannelNum.csv";
    FILE *f_channel=fopen(filePath.c_str(),"r");
    if(!f_channel)
    {
        ROS_ERROR_STREAM(filePath <<" does not exist");
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
    //============================================================
}
void init_setup()
{
    pic.col=0;
    pic.azimuth.resize(POINT_PER_CIRCLE_);
    pic.distance.resize(DATA_NUMBER_PER_SCAN);
    pic.intensity.resize(DATA_NUMBER_PER_SCAN);
    pic.azimuthforeachP.resize(DATA_NUMBER_PER_SCAN);
}

float pixelToDistance(int pixelValue, int passageway)
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

//calibrate intensity
float calibrateIntensity(float intensity,int calIdx, int distance)
{

    int     algDist = 0;
    int     sDist = 0;
    float   realPwr = 0;
    float   refPwr = 0;
    float   tempInten = 0;

    realPwr= intensity;
    if(int(realPwr)<126)
        realPwr=realPwr * 4;
    else
        realPwr=(realPwr-125)*16+500;

    //-------------------------------------------------------------------------------------------------
    //limit sDist belongs to [200,1600] in unit cm
    sDist = (int(distance) > 200) ? int(distance) : 201;
    sDist = (sDist < 1800) ? sDist : 1800;
    //minus the static offset (this data is For the intensity cal useage only)
    algDist = int(sDist - 200) ;
    algDist = algDist < 1400? algDist : 1399;
    refPwr = aIntensityCal[algDist][calIdx];
    tempInten = refPwr / float(realPwr)	;
    tempInten = tempInten * 200		;
    tempInten = tempInten>255?255:tempInten;
    return float(tempInten);
    //------------------------------------------------------------
}

void unpack(const rslidar::rslidarPacket &pkt,pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
    float azimuth;  //0.01 dgree
    float intensity;
    float azimuth_diff;
    float last_azimuth_diff;
    float azimuth_corrected_f;
    int   azimuth_corrected;


    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[42];

    for (int block = 0; block < BLOCKS_PER_PACKET; block++)  //12
    {

        if(UPPER_BANK != raw->blocks[block].header)
        {
            ROS_WARN_STREAM_THROTTLE(60, "skipping invalid RSLIDAR packet: block "
                                     << block << " header value is "
                                     << raw->blocks[block].header);
            return ;
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
            	//ROS_INFO("Error: %d  %d", azi2, azi1);
            	azimuth_diff = 36.0;
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

                azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) / RS16_BLOCK_TDURATION);
                azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;//convert to integral value...
                pic.azimuthforeachP[pic.col*32+k/3]=azimuth_corrected;

                union two_bytes tmp;
                tmp.bytes[1] = raw->blocks[block].data[k];
                tmp.bytes[0] = raw->blocks[block].data[k+1];
                int distance = tmp.uint;// * DISTANCE_RESOLUTION;

                // read intensity
                intensity = raw->blocks[block].data[k+2];
                intensity = calibrateIntensity(intensity,dsr,distance);
		

                float distance2 = pixelToDistance(distance, dsr);
                distance2 = 0.01* distance2;

                pic.distance[pic.col*32+k/3] = distance2;
                pic.intensity[pic.col*32+k/3] = intensity;

            }
        }
        pic.azimuth[pic.col] = azimuth;
        pic.col++;
        
        int azimuth_error = (36000 + (int)round(azimuth - pic.azimuth[0]))%36000;
        if((pic.col>=100) && (azimuth_error<100)) //rotate a full circle
        {
            pointcloud->clear();
            pointcloud->height = RS16_SCANS_PER_FIRING;
            pointcloud->width = 2 * pic.col;
            pointcloud->is_dense = false;
            pointcloud->resize(pointcloud->height * pointcloud->width);
            mat_depth = cv::Mat(cv::Size( 2 * pic.col,RS16_SCANS_PER_FIRING), CV_32FC1, cv::Scalar(0));
            //mat_inten = cv::Mat(cv::Size(2 * pic.col ,RS16_SCANS_PER_FIRING ), CV_8UC1, cv::Scalar(0));
            pointcloud->header.frame_id = "rslidar";
            for (int block_num = 0; block_num < pic.col; block_num++)
            {
                for (int firing = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++)
                {
                    for (int channel = 0; channel < RS16_SCANS_PER_FIRING; channel++)
                    {
                        float dis = pic.distance[block_num * 32 + channel + 16*firing];
                        float arg_horiz = pic.azimuthforeachP[block_num*32 + channel + 16*firing] /18000*CV_PI;
                        float intensity = pic.intensity[block_num*32 + channel + 16*firing];
                        float arg_vert = VERT_ANGLE[channel];
                        pcl::PointXYZI point;
                        if(dis > DISTANCE_MAX || dis < DISTANCE_MIN)  //invalid data
                        {
                            point.x = NAN;
                            point.y = NAN;
                            point.z = NAN;
                            point.intensity = 0;
                            //ROS_INFO("inten: %f", intensity);
                            //pointcloud->push_back(point);
                            pointcloud->at(2*block_num + firing, channel) = point;
                            mat_depth.at<float>(channel,2*block_num + firing) = 0;
                            //mat_inten.at<uchar>(channel,2*block_num + firing) = (uchar)0;
                        }else
                        {
                            point.x = dis * cos(arg_vert) * sin(arg_horiz);
                            point.y = dis * cos(arg_vert) * cos(arg_horiz);
                            point.z = dis * sin(arg_vert);
                            point.intensity = intensity;
                            pointcloud->at(2*block_num + firing, channel) = point;
                            mat_depth.at<float>(channel,2*block_num + firing) = dis;
                            //mat_inten.at<uchar>(channel,2*block_num + firing) = (uchar)intensity;
                            
                        }

                    }
                }
            }
            removeOutlier(pointcloud); //remove noise point

            init_setup();
            pic.header.stamp = pkt.stamp;
        }
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
   PCloudRemove->clear();
    for(int row = 0; row < matMask.rows; row++)
     {
        for(int col = 0 ; col < matMask.cols ; col++ )
        {
            if(matMask.at<uchar>(row,col) != 255)
            {
               //removeNum++;
               PCloudRemove->push_back(pointcloud->at(col,row));
               pointcloud->at(col,row).x =NAN;
               pointcloud->at(col,row).y =NAN;
               pointcloud->at(col,row).z =NAN;
               //pointcloud->at(col,row).intensity= 255;
            }

        }
      }
 // ROS_INFO("PCloudRemove: %d", removeNum);

}
}//namespace rs_driver


