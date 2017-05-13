#include "convertion.h"
namespace rs_driver
{


void loadConfigFile()
{
    /// 读参数文件 2017-02-27
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
            for (int j = 0; j < 16; j++)
            {
                fscanf(f_inten, "%f", &aIntensityCal[loopi - 1][j]);
                // aIntensityCal[loopi - 1][loopj] = a[loopj];
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
    pic.distancenum=0;
    pic.intensitynum=0;
    pic.azimuth.resize(POINT_PER_CIRCLE_);
    pic.distance.resize(DATA_NUMBER_PER_SCAN);
    pic.intensity.resize(DATA_NUMBER_PER_SCAN);
    pic.azimuthforeachP.resize(DATA_NUMBER_PER_SCAN);
}

float pixelToDistance(float pixelValue, int passageway)
{
    float DistanceValue = 0;
    double  deta = 0;

    if (pixelValue < 200 || pixelValue > 11670)
    {
        DistanceValue = 0;
    }
    else
    {
        DistanceValue = (float)pixelValue - g_ChannelNum[passageway];

    }
    return DistanceValue;
}
//校准反射强度值
float calibrateIntensity(float intensity,int calIdx, int distance)
{

    int     algDist = 0;
    int     sDist = 0;
    float   realPwr = 0;
    float   refPwr = 0;
    float   tempInten = 0;

    //---------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------
    //limt intensity data and make it doubled(in FPGA this data is divided by 2)
    /*   intensity = (intensity>3) ? intensity * 2 : 6;
                   realPwr = intensity;*/
    //-------------------------------------------------------------------------------------------------
    //tempInten=intensity ;
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

    for (int block = 0; block < BLOCKS_PER_PACKET; block++, ++pic.col)  //12
    {
        //std::cout<<"block1: "<<block<<std::endl;
        //std::cout<<"pic.col1: "<<pic.col<<std::endl;
        if(UPPER_BANK != raw->blocks[block].header)
        {
            ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                     << block << " header value is "
                                     << raw->blocks[block].header);
            return ;
        }

        azimuth = (float)(256*raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);




        if (block < (BLOCKS_PER_PACKET-1))//12
        {
            int azi1, azi2;
            azi1 = (float)(256*raw->blocks[block+1].rotation_1 + raw->blocks[block+1].rotation_2);
            azi2 = (float)(256*raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);
            azimuth_diff = (float)((36000 + azi1 - azi2)%36000);
            last_azimuth_diff = azimuth_diff;
        }else
        {
            azimuth_diff = last_azimuth_diff;
        }

        for(int firing = 0 ,k = 0;firing < VLP16_FIRINGS_PER_BLOCK; firing++)//2
        {
            for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)//16   3
            {

                // int showPwrs =0 ;
                // int showInten = 0;

                azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
                azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;//convert to integral value...
                pic.azimuthforeachP[pic.col*32+k/3]=azimuth_corrected;

                union two_bytes tmp;
                tmp.bytes[1] = raw->blocks[block].data[k];
                tmp.bytes[0] = raw->blocks[block].data[k+1];
                int distance = tmp.uint;// * DISTANCE_RESOLUTION;


                // 读强度数据
                intensity = raw->blocks[block].data[k+2];
                //  intensity = calibrateIntensity(intensity,dsr,distance);

                float distance2 = pixelToDistance(distance, dsr);
                distance2 = 0.01* distance2;

                pic.distance[pic.col*32+k/3] = distance2;
                pic.distancenum++;
                pic.intensity[pic.col*32+k/3] = intensity;
                pic.intensitynum++;


            }
        }


        if((pic.col>=100) && (abs(azimuth-pic.azimuth[0])<100)) //旋转完整一圈
        {
            //std::cout<<"pic.col: "<<pic.col<<std::endl;
            //std::cout<<"abs(azimuth-pic.azimuth[0]: "<<azimuth-pic.azimuth[0]<<std::endl;
            pointcloud->clear();
            pointcloud->height = VLP16_SCANS_PER_FIRING;
            pointcloud->width = 2 * pic.col;
            pointcloud->is_dense = true;
            pointcloud->resize(pointcloud->height * pointcloud->width);
            mat_depth = cv::Mat(cv::Size( 2 * pic.col,VLP16_SCANS_PER_FIRING), CV_32FC1, cv::Scalar(0));
            mat_inten = cv::Mat(cv::Size(2 * pic.col ,VLP16_SCANS_PER_FIRING ), CV_8UC1, cv::Scalar(0));;
            pointcloud->header.frame_id = "rslidar";
            for (int block = 0; block < pic.col; block++)
            {
                for (int firing = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
                {
                    for (int channel = 0; channel < VLP16_SCANS_PER_FIRING; channel++)
                    {
                        float dis = pic.distance[block * 32 + a_channelOrder[channel] + 16*firing];
                        float arg_horiz = pic.azimuthforeachP[block*32 + a_channelOrder[channel] + 16*firing] /18000*CV_PI;
                        float intensity = pic.intensity[block*32 + a_channelOrder[channel] + 16*firing];
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
                            pointcloud->at(2*block + firing, channel) = point;
                            mat_depth.at<float>(channel,2*block + firing) = 0;
                            mat_inten.at<uchar>(channel,2*block + firing) = (uchar)0;
                        }else
                        {
                            point.x = dis * cos(arg_vert) * sin(arg_horiz);
                            point.y = dis * cos(arg_vert) * cos(arg_horiz);
                            point.z = dis * sin(arg_vert);
                            point.intensity = intensity;
                            //ROS_INFO("inten: %f", intensity);
                            //pointcloud->push_back(point);
                            pointcloud->at(2*block + firing, channel) = point;
                            mat_depth.at<float>(channel,2*block + firing) = dis;
                            mat_inten.at<uchar>(channel,2*block + firing) = (uchar)intensity;
                        }

                    }
                }
            }
             removeOutlier(pointcloud); //去除脏数据
            //std::cout<<"pointcloud->width: "<<pointcloud->width<<std::endl;
            //std::cout<<"pic.col: "<<pic.col<<std::endl;
/*
            debug_a++;
            if(debug_a%100 == 0)
            {   std::string name;
                std::stringstream strStream;
                strStream <<"/home/guoleiming/"<< (debug_a);

                name = strStream.str() +".bmp";
                if( cv::imwrite(name,mat_inten))
                {
                    std::cout << "save file " << name << std::endl;
                }
                name = strStream.str() +".pcd";
                pcl::io::savePCDFileASCII(name,*pointcloud);

            }

*/
            init_setup();
            //std::cout<<"pic.col: "<<pic.col<<std::endl;S
            pic.header.stamp = pkt.stamp;
        }

        pic.azimuth[pic.col] = azimuth;

    }
}



    /***************************************************************************************
    Function:  区域生长算法
    Input:     src 待处理原图像 pt 初始生长点 th 生长的阈值条件
    Output:    肺实质的所在的区域 实质区是白色，其他区域是黑色
    Description: 生长结果区域标记为白色(255),背景色为黑色(0)
    Return:    Mat
    Others:    NULL
    ***************************************************************************************/
   void  regionGrow(cv::Mat src,cv::Mat &matDst, cv::Point2i pt, int th)
    {

        cv::Point2i ptGrowing;                      //待生长点位置
        int nGrowLable = 0;                             //标记是否生长过
        int nSrcValue = 0;                              //生长起点灰度值
        int nCurValue = 0;                              //当前生长点灰度值

        //生长方向顺序数据
        int DIR[8][2] = {{-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}};
        cv::Vector<cv::Point2i> vcGrowPt;                     //生长点栈
        vcGrowPt.push_back(pt);                         //将生长点压入栈中
       // matDst.at<uchar>(pt) = 255;               //标记生长点
        nSrcValue = src.at<float>(pt);            //记录生长点的灰度值

       while (!vcGrowPt.empty())                       //生长栈不为空则生长
        {
            pt = vcGrowPt.back();                       //取出一个生长点
            vcGrowPt.pop_back();

            //分别对八个方向上的点进行生长
            for (int i = 0; i<8; ++i)
            {
                ptGrowing.x = pt.x + DIR[i][0];
                ptGrowing.y = pt.y + DIR[i][1];
                //检查是否是边缘点
                if (ptGrowing.x < 0 || ptGrowing.y < 0 || ptGrowing.x > (src.cols-1) || (ptGrowing.y > src.rows -1))
                    continue;

                nGrowLable = matDst.at<uchar>(ptGrowing);      //当前待生长点的灰度值

                if (nGrowLable == 0)                    //如果标记点还没有被生长
                {
                    nCurValue = src.at<float>(ptGrowing);
                    if (abs(nSrcValue - nCurValue) < th)                 //在阈值范围内则生长
                    {
                        matDst.at<uchar>(ptGrowing) = 255;     //标记为白色
                        vcGrowPt.push_back(ptGrowing);                  //将下一个生长点压入栈中
                    }
                }
            }
        }

    }



void removeOutlier(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud )
{

  // std::map<cv::Point,float> map_outlier;
   cv::Mat matMask = cv::Mat::zeros(mat_depth.size(), CV_8UC1);   //创建一个空白区域，填充为黑色
   //cv::Mat matDiff = cv::Mat(mat_depth.size(), CV_32FC1, cv::Scalar(0));
int removeNum =0;
   for(int row = 1; row < mat_depth.rows-1; row++)  //mask 3*3
     {
        for(int col = 1 ; col < mat_depth.cols-1 ; col++ )
       {
          float d = mat_depth.at<float>(row,col);
          if( d > 30 || d <0.2)
          {
              matMask.at<uchar>(row,col) = (uchar)255;
              continue;
          }
          cv::Point2i pt(col,row);
           regionGrow(mat_depth,matMask,pt,1);
       }
    }

    PCloudRemove->clear();
    for(int row = 1; row < matMask.rows-1; row++)  //mask 3*3
     {
        for(int col = 1 ; col < matMask.cols-1 ; col++ )
        {
            if(matMask.at<uchar>(row,col) == 0)
            {
               removeNum++;
               PCloudRemove->push_back(pointcloud->at(col,row));
               pointcloud->at(col,row).x =NAN;
               pointcloud->at(col,row).y =NAN;
               pointcloud->at(col,row).z =NAN;
               //pointcloud->at(col,row).intensity= 255;
            }

        }
      }

    std::cout << "remove " << removeNum << " outlier" <<std::endl;


}
}//namespace rs_driver


