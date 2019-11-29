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
RawData::RawData()
{
  this->is_init_angle_ = false;
  this->is_init_curve_ = false;
  this->is_init_top_fw_ = false;
}

void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string anglePath, curvesPath, channelPath, curvesRatePath;
  std::string model;
  std::string sub_model;
  std::string resolution_param;
  private_nh.param("model", model, std::string("RS16"));
  private_nh.param("curves_path", curvesPath, std::string(""));
  private_nh.param("angle_path", anglePath, std::string(""));
  private_nh.param("channel_path", channelPath, std::string(""));
  private_nh.param("curves_rate_path", curvesRatePath, std::string(""));
  private_nh.param("start_angle", start_angle_, int(0));
  private_nh.param("end_angle", end_angle_, int(360));

  ROS_INFO_STREAM("[cloud][rawdata] lidar model: " << model);
  if (start_angle_ < 0 || start_angle_ > 360 || end_angle_ < 0 || end_angle_ > 360)
  {
    start_angle_ = 0;
    end_angle_ = 360;
    ROS_INFO_STREAM("[cloud][rawdata] start and end angle select feature deactivated.");
  }
  else
  {
    ROS_INFO_STREAM("[cloud][rawdata] start and end angle feature activated.");
  }

  angle_flag_ = true;
  if (start_angle_ > end_angle_)
  {
    angle_flag_ = false;
    //    ROS_INFO_STREAM("[cloud][rawdata] Start angle is smaller than end angle, not the normal state!");
  }

  ROS_INFO_STREAM("[cloud][rawdata] start_angle: " << start_angle_ << " end_angle: " << end_angle_
                                                   << " angle_flag: " << angle_flag_);
  start_angle_ = start_angle_ * 100;
  end_angle_ = end_angle_ * 100;

  private_nh.param("max_distance", max_distance_, 200.0f);
  private_nh.param("min_distance", min_distance_, 0.4f);

  ROS_INFO_STREAM("[cloud][rawdata] distance threshlod, max: " << max_distance_ << " m, min: " << min_distance_
                                                               << " m");

  intensity_mode_ = 3;
  info_print_flag_ = false;
  private_nh.param("resolution_type", resolution_param, std::string("0.5cm"));
  private_nh.param("intensity_mode", intensity_mode_, 1);

  if (resolution_param == "0.5cm")
  {
    dis_resolution_mode_ = 0;
  }
  else
  {
    dis_resolution_mode_ = 1;
  }

  ROS_INFO_STREAM("[cloud][rawdata] initialize resolution type: " << (dis_resolution_mode_ ? "1 cm" : "0.5 cm")
                                                                  << ", intensity mode: " << intensity_mode_);

  if (model == "RS16")
  {
    numOfLasers = 16;
    Rx_ = 0.03825;
    Ry_ = -0.01088;
    Rz_ = 0;
  }
  else if (model == "RS32")
  {
    numOfLasers = 32;
    TEMPERATURE_RANGE = 50;
    Rx_ = 0.03997;
    Ry_ = -0.01087;
    Rz_ = 0;
    isBpearlLidar_ = false;
  }
  else if(model == "RSBPEARL")
  {
    numOfLasers = 32;
    TEMPERATURE_RANGE = 50;
    Rx_ = 0.01697;
    Ry_ = -0.0085;
    Rz_ = 0.12644;
    isBpearlLidar_ = true;
  }
  else if (model == "RSBPEARL_MINI")
  {
    numOfLasers = 32;
    TEMPERATURE_RANGE = 50;
    Rx_ = 0.01473;
    Ry_ = 0.0085;
    Rz_ = 0.09427;
    isBpearlLidar_ = true;
  }
  else
  {
    std::cout << "Bad model!" << std::endl;
  }

  intensityFactor = 51;

  // return mode default
  return_mode_ = 1;

  // lookup table init
  this->cos_lookup_table_.resize(36000);
  this->sin_lookup_table_.resize(36000);
  for (unsigned int i = 0; i < 36000; i++)
  {
    double rad = RS_TO_RADS(i / 100.0f);

    this->cos_lookup_table_[i] = std::cos(rad);
    this->sin_lookup_table_[i] = std::sin(rad);
  }

  /// 读参数文件 2017-02-27
  FILE* f_inten = fopen(curvesPath.c_str(), "r");
  int loopi = 0;
  int loopj = 0;
  int loop_num;
  if (!f_inten)
  {
    ROS_WARN_STREAM("[cloud][rawdata] " << curvesPath << " does not exist");
  }
  else
  {
    fseek(f_inten, 0, SEEK_END);  //定位到文件末
    int size = ftell(f_inten);    //文件长度

    if (size > 10000)  //老版的curve
    {
      Curvesis_new = false;
      loop_num = 1600;
    }
    else
    {
      Curvesis_new = true;
      loop_num = 7;
    }
    fseek(f_inten, 0, SEEK_SET);
    while (!feof(f_inten))
    {
      float a[32];
      loopi++;

      if (loopi > loop_num)
        break;
      if (numOfLasers == 16)
      {
        int tmp = fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &a[0], &a[1], &a[2], &a[3],
                         &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15]);
      }
      else if (numOfLasers == 32)
      {
        int tmp = fscanf(f_inten,
                         "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                         "%f,%f,%f,%f\n",
                         &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
                         &a[13], &a[14], &a[15], &a[16], &a[17], &a[18], &a[19], &a[20], &a[21], &a[22], &a[23], &a[24],
                         &a[25], &a[26], &a[27], &a[28], &a[29], &a[30], &a[31]);
      }
      if (Curvesis_new)
      {
        for (loopj = 0; loopj < numOfLasers; loopj++)
        {
          aIntensityCal[loopi - 1][loopj] = a[loopj];
        }
      }
      else
      {
        for (loopj = 0; loopj < numOfLasers; loopj++)
        {
          aIntensityCal_old[loopi - 1][loopj] = a[loopj];
        }
      }
      // ROS_INFO_STREAM("new is " << a[0]);
    }
    fclose(f_inten);
  }
  //=============================================================
  FILE* f_angle = fopen(anglePath.c_str(), "r");
  if (!f_angle)
  {
    ROS_WARN_STREAM("[cloud][rawdata] " << anglePath << " does not exist");
  }
  else
  {
    float b[32], d[32];
    int loopk = 0;
    int loopn = 0;
    while (!feof(f_angle))
    {
      int tmp = fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
      loopk++;
      if (loopk > (numOfLasers - 1))
        break;
    }
    for (loopn = 0; loopn < numOfLasers; loopn++)
    {
      VERT_ANGLE[loopn] = b[loopn] * 100;
      HORI_ANGLE[loopn] = d[loopn] * 100;
    }

    if (numOfLasers == 16)
    {
      memset(HORI_ANGLE, 0, sizeof(HORI_ANGLE));
    }

    fclose(f_angle);
  }

  //=============================================================
  FILE* f_channel = fopen(channelPath.c_str(), "r");
  if (!f_channel)
  {
    ROS_WARN_STREAM("[cloud][rawdata] " << channelPath << " does not exist");
  }
  else
  {
    int loopl = 0;
    int loopm = 0;
    int c[51];
    int tempMode = 1;
    while (!feof(f_channel))
    {
      if (numOfLasers == 16)
      {
        int tmp = fscanf(f_channel,
                         "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                         "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                         &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                         &c[13], &c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
                         &c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36],
                         &c[37], &c[38], &c[39], &c[40]);
      }
      else
      {
        int tmp = fscanf(f_channel,
                         "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                         "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                         &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                         &c[13], &c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
                         &c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36],
                         &c[37], &c[38], &c[39], &c[40], &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48],
                         &c[49], &c[50]);
      }
      for (loopl = 0; loopl < TEMPERATURE_RANGE + 1; loopl++)
      {
        g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
      }
      loopm++;
      if (loopm > (numOfLasers - 1))
      {
        break;
      }
    }
    fclose(f_channel);
  }

  if (numOfLasers == 32)
  {
    FILE* f_curvesRate = fopen(curvesRatePath.c_str(), "r");
    if (!f_curvesRate)
    {
      ROS_WARN_STREAM("[cloud][rawdata] " << curvesRatePath << " does not exist");
      for (int i = 0; i < 32; ++i)
      {
        CurvesRate[i] = 1.0;
      }
    }
    else
    {
      int loopk = 0;
      while (!feof(f_curvesRate))
      {
        int tmp = fscanf(f_curvesRate, "%f\n", &CurvesRate[loopk]);
        loopk++;
        if (loopk > (numOfLasers - 1))
          break;
      }
      fclose(f_curvesRate);
    }
  }

  // receive difop data
  // subscribe to difop rslidar packets, if not right correct data in difop, it will not revise the correct data in the
  // VERT_ANGLE, HORI_ANGLE etc.
  difop_sub_ = node.subscribe("rslidar_packets_difop", 10, &RawData::processDifop, (RawData*)this);
}

void RawData::processDifop(const rslidar_msgs::rslidarPacket::ConstPtr& difop_msg)
{
  // std::cout << "Enter difop callback!" << std::endl;
  const uint8_t* data = &difop_msg->data[0];
  bool is_support_dual_return = false;

  // check header
  if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a)
  {
    return;
  }

  // uint8_t save_return_mode = return_mode_;
  // return mode check
  if ((data[45] == 0x08 && data[46] == 0x02 && data[47] >= 0x09) || (data[45] > 0x08) ||
      (data[45] == 0x08 && data[46] > 0x02))
  {
    is_support_dual_return = true;
    if (data[300] == 0x01 || data[300] == 0x02)
    {
      return_mode_ = data[300];
    }
    else
    {
      return_mode_ = 0;
    }
  }
  else
  {
    is_support_dual_return = false;
    return_mode_ = 1;
  }
  //
  if (!this->is_init_top_fw_)
  {
    if ((data[41] == 0x00 && data[42] == 0x00 && data[43] == 0x00) ||
        (data[41] == 0xff && data[42] == 0xff && data[43] == 0xff) ||
        (data[41] == 0x55 && data[42] == 0xaa && data[43] == 0x5a) ||
        (data[41] == 0xe9 && data[42] == 0x01 && data[43] == 0x00))
    {
      dis_resolution_mode_ = 1;  // 1cm resolution
      // std::cout << "The distance resolution is 1cm" << std::endl;
    }
    else
    {
      dis_resolution_mode_ = 0;  // 0.5cm resolution
      // std::cout << "The distance resolution is 0.5cm" << std::endl;
    }
    this->is_init_top_fw_ = true;
  }

  if (!this->is_init_curve_)
  {
    bool curve_flag = true;
    // check difop reigon has beed flashed the right data
    if ((data[50] == 0x00 || data[50] == 0xff) && (data[51] == 0x00 || data[51] == 0xff) &&
        (data[52] == 0x00 || data[52] == 0xff) && (data[53] == 0x00 || data[53] == 0xff))
    {
      curve_flag = false;
    }

    // TODO check why rsview here no 32 laser, be more careful the new, old version
    // Init curves
    if (curve_flag)
    {
      unsigned char checkbit;
      int bit1, bit2;
      for (int loopn = 0; loopn < numOfLasers; ++loopn)
      {
        // check the curves' parameter in difop
        checkbit = *(data + 50 + loopn * 15) ^ *(data + 50 + loopn * 15 + 1);
        for (int loopm = 1; loopm < 7; ++loopm)
        {
          checkbit = checkbit ^ (*(data + 50 + loopn * 15 + loopm * 2)) ^ (*(data + 50 + loopn * 15 + loopm * 2 + 1));
        }
        if (checkbit != *(data + 50 + loopn * 15 + 14))
        {
          return;
        }
      }
      for (int loopn = 0; loopn < numOfLasers; ++loopn)
      {
        // calculate curves' parameters
        bit1 = static_cast<int>(*(data + 50 + loopn * 15));
        bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 1));
        aIntensityCal[0][loopn] = (bit1 * 256 + bit2) * 0.001;
        bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 2));
        bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 3));
        aIntensityCal[1][loopn] = (bit1 * 256 + bit2) * 0.001;
        bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 4));
        bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 5));
        aIntensityCal[2][loopn] = (bit1 * 256 + bit2) * 0.001;
        bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 6));
        bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 7));
        aIntensityCal[3][loopn] = (bit1 * 256 + bit2) * 0.001;
        bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 8));
        bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 9));
        aIntensityCal[4][loopn] = (bit1 * 256 + bit2) * 0.00001;
        bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 10));
        bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 11));
        aIntensityCal[5][loopn] = -(bit1 * 256 + bit2) * 0.0001;
        bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 12));
        bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 13));
        aIntensityCal[6][loopn] = (bit1 * 256 + bit2) * 0.001;
      }
      this->is_init_curve_ = true;
      ROS_INFO_STREAM("[cloud][rawdata] curves data is wrote in difop packet!");
      Curvesis_new = true;
    }

    if ((data[290] != 0x00) && (data[290] != 0xff))
    {
      intensityFactor = static_cast<int>(*(data + 290));  // intensity factor introduced since than 20181115
    }

    if ((data[291] == 0x00) || (data[291] == 0xff) || (data[291] == 0xa1))
    {
      intensity_mode_ = 1;  // mode for the top firmware lower than T6R23V8(16) or T9R23V6(32)
    }
    else if (data[291] == 0xb1)
    {
      intensity_mode_ = 2;  // mode for the top firmware higher than T6R23V8(16) or T9R23V6(32)
    }
    else if (data[291] == 0xc1)
    {
      intensity_mode_ = 3;  // mode for the top firmware higher than T6R23V9
    }
    
  }

  if (!this->is_init_angle_)
  {
    // check header
    {
      bool angle_flag = true;
      if (numOfLasers == 16)
      {
        // check difop reigon has beed flashed the right data
        if ((data[1165] == 0x00 || data[1165] == 0xff) && (data[1166] == 0x00 || data[1166] == 0xff) &&
            (data[1167] == 0x00 || data[1167] == 0xff) && (data[1168] == 0x00 || data[1168] == 0xff))
        {
          angle_flag = false;
        }
      }
      else if (numOfLasers == 32)
      {
        if ((data[468] == 0x00 || data[468] == 0xff) && (data[469] == 0x00 || data[469] == 0xff) &&
            (data[470] == 0x00 || data[470] == 0xff))
        {
          angle_flag = false;
        }
      }

      // angle
      if (angle_flag)
      {
        // TODO check the HORI_ANGLE
        int bit1, bit2, bit3, symbolbit;
        if (numOfLasers == 16)
        {
          for (int loopn = 0; loopn < 16; ++loopn)
          {
            if (loopn < 8)
            {
              symbolbit = -1;
            }
            else
            {
              symbolbit = 1;
            }
            bit1 = static_cast<int>(*(data + 1165 + loopn * 3));
            bit2 = static_cast<int>(*(data + 1165 + loopn * 3 + 1));
            bit3 = static_cast<int>(*(data + 1165 + loopn * 3 + 2));
            VERT_ANGLE[loopn] = (bit1 * 256 * 256 + bit2 * 256 + bit3) * symbolbit * 0.01f;
            // std::cout << VERT_ANGLE[loopn] << std::endl;
            // TODO
            HORI_ANGLE[loopn] = 0;
          }
        }
        else if (numOfLasers == 32)
        {
          for (int loopn = 0; loopn < 32; ++loopn)
          {
            // vertical angle
            bit1 = static_cast<int>(*(data + 468 + loopn * 3));
            if (bit1 == 0)
              symbolbit = 1;
            else if (bit1 == 1)
              symbolbit = -1;
            bit2 = static_cast<int>(*(data + 468 + loopn * 3 + 1));
            bit3 = static_cast<int>(*(data + 468 + loopn * 3 + 2));
            if (isBpearlLidar_)
              VERT_ANGLE[loopn] = (bit2 * 256 + bit3) * symbolbit * 0.01f * 100;
            else
              VERT_ANGLE[loopn] = (bit2 * 256 + bit3) * symbolbit * 0.001f * 100;
            // horizontal offset angle
            bit1 = static_cast<int>(*(data + 564 + loopn * 3));
            if (bit1 == 0)
              symbolbit = 1;
            else if (bit1 == 1)
              symbolbit = -1;
            bit2 = static_cast<int>(*(data + 564 + loopn * 3 + 1));
            bit3 = static_cast<int>(*(data + 564 + loopn * 3 + 2));
            HORI_ANGLE[loopn] = (bit2 * 256 + bit3) * symbolbit * 0.001f * 100;
          }
        }

        this->is_init_angle_ = true;
        ROS_INFO_STREAM("[cloud][rawdata] angle data is wrote in difop packet!");
      }
    }
  }

  if (!info_print_flag_)
  {
    info_print_flag_ = true;

    // print info
    float prin_dis;
    if (dis_resolution_mode_ == 0)
    {
      prin_dis = 0.5;
    }
    else
    {
      prin_dis = 1;
    }
    ROS_INFO_STREAM("[cloud][rawdata] distance resolution is: " << prin_dis << " cm, intensity mode is: Mode "
                                                                << intensity_mode_);
    if (is_support_dual_return == false)
    {
      ROS_INFO_STREAM("[cloud][rawdata] lidar only support single return wave!");
    }
    else
    {
      if (return_mode_ == 0)
      {
        ROS_INFO_STREAM("[cloud][rawdata] lidar support dual return wave, the current mode is: dual");
      }
      else if (return_mode_ == 1)
      {
        ROS_INFO_STREAM("[cloud][rawdata] lidar support dual return wave, the current mode is: strongest");
      }
      else if (return_mode_ == 2)
      {
        ROS_INFO_STREAM("[cloud][rawdata] lidar support dual return wave, the current mode is: last");
      }
    }

    ROS_INFO_STREAM("[cloud][rawdata] difop intensity mode: "<<intensity_mode_);
  }
}

float RawData::pixelToDistance(int pixelValue, int passageway)
{
  float DistanceValue;
  int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
  if (pixelValue <= g_ChannelNum[passageway][indexTemper])
  {
    DistanceValue = 0.0;
  }
  else
  {
    DistanceValue = (float)(pixelValue - g_ChannelNum[passageway][indexTemper]);
  }
  return DistanceValue;
}

int RawData::correctAzimuth(float azimuth_f, int passageway)
{
  int azimuth;
  if (azimuth_f > 0.0 && azimuth_f < 3000.0)
  {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
  }
  else
  {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway];
  }
  azimuth = (int)azimuth_f;
  azimuth = ((azimuth%36000)+36000)%36000;

  return azimuth;
}

//------------------------------------------------------------
//校准反射强度值
float RawData::calibrateIntensity(float intensity, int calIdx, int distance)
{
  if (intensity_mode_ == 3)
  {
    return intensity;
  }
  else
  {
    int algDist;
    int sDist;
    float realPwr;
    float refPwr;
    float tempInten;
    float distance_f;
    float endOfSection1, endOfSection2;
    int temp = estimateTemperature(temper);

    realPwr = std::max((float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);

    if (intensity_mode_ == 1)
    {
      // transform the one byte intensity value to two byte
      if ((int)realPwr < 126)
        realPwr = realPwr * 4.0f;
      else if ((int)realPwr >= 126 && (int)realPwr < 226)
        realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
      else
        realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;
    }
    else if (intensity_mode_ == 2)
    {
      // the caculation for the firmware after T6R23V8(16) and T9R23V6(32)
      if ((int)realPwr < 64)
        realPwr = realPwr;
      else if ((int)realPwr >= 64 && (int)realPwr < 176)
        realPwr = (realPwr - 64.0f) * 4.0f + 64.0f;
      else
        realPwr = (realPwr - 176.0f) * 16.0f + 512.0f;
    }
    else
    {
      ROS_WARN("[cloud][rawdata] The intensity mode is not right");
    }

    int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;

    // limit sDist
    sDist = (distance > g_ChannelNum[calIdx][indexTemper]) ? distance : g_ChannelNum[calIdx][indexTemper];

    // minus the static offset (this data is For the intensity cal useage only)
    algDist = sDist - g_ChannelNum[calIdx][indexTemper];
    if (dis_resolution_mode_ == 0)
    {
      distance_f = (float)algDist * DISTANCE_RESOLUTION_NEW;
    }
    else
    {
      distance_f = (float)algDist * DISTANCE_RESOLUTION;
    }
    distance_f = (distance_f > this->max_distance_) ? this->max_distance_ : distance_f;

    // calculate intensity ref curves
    float refPwr_temp = 0.0f;
    int order = 3;
    endOfSection1 = 5.0f;
    endOfSection2 = 40.0;

    if (intensity_mode_ == 1)
    {
      if (distance_f <= endOfSection1)
      {
        refPwr_temp = aIntensityCal[0][calIdx] * exp(aIntensityCal[1][calIdx] - aIntensityCal[2][calIdx] * distance_f) +
                      aIntensityCal[3][calIdx];
      }
      else
      {
        for (int i = 0; i < order; i++)
        {
          refPwr_temp += aIntensityCal[i + 4][calIdx] * (pow(distance_f, order - 1 - i));
        }
      }
    }
    else if (intensity_mode_ == 2)
    {
      if (distance_f <= endOfSection1)
      {
        refPwr_temp = aIntensityCal[0][calIdx] * exp(aIntensityCal[1][calIdx] - aIntensityCal[2][calIdx] * distance_f) +
                      aIntensityCal[3][calIdx];
      }
      else if (distance_f > endOfSection1 && distance_f <= endOfSection2)
      {
        for (int i = 0; i < order; i++)
        {
          refPwr_temp += aIntensityCal[i + 4][calIdx] * (pow(distance_f, order - 1 - i));
        }
      }
      else
      {
        float refPwr_temp0 = 0.0f;
        float refPwr_temp1 = 0.0f;
        for (int i = 0; i < order; i++)
        {
          refPwr_temp0 += aIntensityCal[i + 4][calIdx] * (pow(40.0f, order - 1 - i));
          refPwr_temp1 += aIntensityCal[i + 4][calIdx] * (pow(39.0f, order - 1 - i));
        }
        refPwr_temp = 0.3f * (refPwr_temp0 - refPwr_temp1) * distance_f + refPwr_temp0;
      }
    }
    else
    {
      ROS_WARN("[cloud][rawdata] The intensity mode is not right");
    }

    refPwr = std::max(std::min(refPwr_temp, 500.0f), 4.0f);

    tempInten = (intensityFactor * refPwr) / realPwr;
    if (numOfLasers == 32)
    {
      tempInten = tempInten * CurvesRate[calIdx];
    }
    tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
    return tempInten;
  }
}

//------------------------------------------------------------
//校准反射强度值 old
float RawData::calibrateIntensity_old(float intensity, int calIdx, int distance)
{
  int algDist;
  int sDist;
  int uplimitDist;
  float realPwr;
  float refPwr;
  float tempInten;

  int temp = estimateTemperature(temper);
  realPwr = std::max((float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);
  // realPwr = intensity;

  if ((int)realPwr < 126)
    realPwr = realPwr * 4.0f;
  else if ((int)realPwr >= 126 && (int)realPwr < 226)
    realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
  else
    realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;

  int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
  uplimitDist = g_ChannelNum[calIdx][indexTemper] + 1400;
  sDist = (distance > g_ChannelNum[calIdx][indexTemper]) ? distance : g_ChannelNum[calIdx][indexTemper];
  sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
  // minus the static offset (this data is For the intensity cal useage only)
  algDist = sDist - g_ChannelNum[calIdx][indexTemper];
  // algDist = algDist < 1400? algDist : 1399;
  refPwr = aIntensityCal_old[algDist][calIdx];

  tempInten = (51 * refPwr) / realPwr;
  if (numOfLasers == 32)
  {
    tempInten = tempInten * CurvesRate[calIdx];
  }
  tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
  return tempInten;
}

//------------------------------------------------------------
int RawData::isABPacket(int distance)
{
  int ABflag = 0;
  if ((distance & 32768) != 0)
  {
    ABflag = 1;  // B
  }
  else
  {
    ABflag = 0;  // A
  }
  return ABflag;
}

//------------------------------------------------------------
float RawData::computeTemperature(unsigned char bit1, unsigned char bit2)
{
  float Temp;
  float bitneg = bit2 & 128;   // 10000000
  float highbit = bit2 & 127;  // 01111111
  float lowbit = bit1 >> 3;
  if (bitneg == 128)
  {
    Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
  }
  else
  {
    Temp = (highbit * 32 + lowbit) * 0.0625f;
  }

  return Temp;
}

//------------------------------------------------------------
int RawData::estimateTemperature(float Temper)
{
  int temp = (int)floor(Temper + 0.5);
  if (temp < TEMPERATURE_MIN)
  {
    temp = TEMPERATURE_MIN;
  }
  else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE)
  {
    temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
  }

  return temp;
}
//------------------------------------------------------------

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  // check pkt header
  if (pkt.data[0] != 0x55 || pkt.data[1] != 0xAA || pkt.data[2] != 0x05 || pkt.data[3] != 0x0A)
  {
    return;
  }

  if (numOfLasers == 32)
  {
    unpack_RS32(pkt, pointcloud);
    return;
  }
  float azimuth;  // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[42];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
  {
    if (UPPER_BANK != raw->blocks[block].header)
    {
      ROS_INFO_STREAM_THROTTLE(180, "[cloud][rawdata] skipping RSLIDAR DIFOP packet");
      break;
    }

    if (tempPacketNum < 20000 && tempPacketNum > 0)  // update temperature information per 20000 packets
    {
      tempPacketNum++;
    }
    else
    {
      temper = computeTemperature(pkt.data[38], pkt.data[39]);
      tempPacketNum = 1;
    }

    azimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

    int azi1, azi2;
    if (block < (BLOCKS_PER_PACKET - 1))  // 12
    {
      // int azi1, azi2;
      azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
      azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
    }
    else
    {
      
      azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
      
    }
    uint16_t diff = (36000 + azi1 - azi2) % 36000;
    if (diff > 100)  //to avoid when the lidar is set to specific FOV that cause the big difference between angle
    {
      diff = 0;
    }
    azimuth_diff = (float)(diff);

    for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++)  // 2
    {
      for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
      {
        if (0 == return_mode_)
        {
          azimuth_corrected_f = azimuth + (azimuth_diff * (dsr * RS16_DSR_TOFFSET)) / RS16_FIRING_TOFFSET;
        }
        else
        {
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) /
                                           RS16_BLOCK_TDURATION);
        }
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;  // convert to integral value...

        union two_bytes tmp;
        tmp.bytes[1] = raw->blocks[block].data[k];
        tmp.bytes[0] = raw->blocks[block].data[k + 1];
        int distance = tmp.uint;

        // read intensity
        intensity = raw->blocks[block].data[k + 2];
        if (Curvesis_new)
          intensity = calibrateIntensity(intensity, dsr, distance);
        else
          intensity = calibrateIntensity_old(intensity, dsr, distance);

        float distance2 = pixelToDistance(distance, dsr);
        if (dis_resolution_mode_ == 0)  // distance resolution is 0.5cm
        {
          distance2 = distance2 * DISTANCE_RESOLUTION_NEW;
        }
        else
        {
          distance2 = distance2 * DISTANCE_RESOLUTION;
        }

        int arg_horiz = (azimuth_corrected + 36000) % 36000;
        int arg_horiz_orginal = arg_horiz;
        int arg_vert = ((VERT_ANGLE[dsr]) % 36000 + 36000) % 36000;

        pcl::PointXYZI point;

        if (distance2 > max_distance_ || distance2 < min_distance_ ||
            (angle_flag_ && (arg_horiz < start_angle_ || arg_horiz > end_angle_)) ||
            (!angle_flag_ && (arg_horiz > end_angle_ && arg_horiz < start_angle_)))  // invalid distance
        {
          point.x = NAN;
          point.y = NAN;
          point.z = NAN;
          point.intensity = 0;
          pointcloud->at(2 * this->block_num + firing, dsr) = point;
        }
        else
        {
          // If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below

          point.x = distance2 * this->cos_lookup_table_[arg_vert] * this->cos_lookup_table_[arg_horiz] +
                    Rx_ * this->cos_lookup_table_[arg_horiz_orginal];
          point.y = -distance2 * this->cos_lookup_table_[arg_vert] * this->sin_lookup_table_[arg_horiz] -
                    Rx_ * this->sin_lookup_table_[arg_horiz_orginal];
          point.z = distance2 * this->sin_lookup_table_[arg_vert] + Rz_;
          point.intensity = intensity;
          pointcloud->at(2 * this->block_num + firing, dsr) = point;
        }
      }
    }
  }
}

void RawData::unpack_RS32(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  float azimuth;  // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[42];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
  {
    if (UPPER_BANK != raw->blocks[block].header)
    {
      ROS_INFO_STREAM_THROTTLE(180, "[cloud][rawdata] skipping RSLIDAR DIFOP packet");
      break;
    }

    if (tempPacketNum < 20000 && tempPacketNum > 0)  // update temperature information per 20000 packets
    {
      tempPacketNum++;
    }
    else
    {
      temper = computeTemperature(pkt.data[38], pkt.data[39]);
      // ROS_INFO_STREAM("Temp is: " << temper);
      tempPacketNum = 1;
    }

    azimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

    int azi1, azi2;
    if (0 == return_mode_)
    {                                       // dual return mode
      if (block < (BLOCKS_PER_PACKET - 2))  // 12
      {
        azi1 = 256 * raw->blocks[block + 2].rotation_1 + raw->blocks[block + 2].rotation_2;
        azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      }
      else
      {
        azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
        azi2 = 256 * raw->blocks[block - 2].rotation_1 + raw->blocks[block - 2].rotation_2;
      }
    }
    else
    {
      if (block < (BLOCKS_PER_PACKET - 1))  // 12
      {
        azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
        azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      }
      else
      {
        azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
        azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
      }
    }
    uint16_t diff = (36000 + azi1 - azi2) % 36000;
    if (diff > 100)  //to avoid when the lidar is set to specific FOV that cause the big difference between angle
    {
      diff = 0;
    }
    azimuth_diff = (float)(diff);

    if (dis_resolution_mode_ == 0)  // distance resolution is 0.5cm and delete the AB packet mechanism
    {
      for (int dsr = 0, k = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++, k += RAW_SCAN_SIZE)  // 16 3
      {
        int dsr_temp;
        if (dsr >= 16)
        {
          dsr_temp = dsr - 16;
        }
        else
        {
          dsr_temp = dsr;
        }
        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr_temp * RS32_DSR_TOFFSET)) / RS32_BLOCK_TDURATION);
        azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);

        union two_bytes tmp;
        tmp.bytes[1] = raw->blocks[block].data[k];
        tmp.bytes[0] = raw->blocks[block].data[k + 1];
        int distance = tmp.uint;

        // read intensity
        intensity = (float)raw->blocks[block].data[k + 2];
        intensity = calibrateIntensity(intensity, dsr, distance);

        float distance2 = pixelToDistance(distance, dsr);
        distance2 = distance2 * DISTANCE_RESOLUTION_NEW;

        int arg_horiz_orginal = (int)azimuth_corrected_f % 36000;
        int arg_horiz = azimuth_corrected;
        int arg_vert = ((VERT_ANGLE[dsr]) % 36000 + 36000) % 36000;
        pcl::PointXYZI point;

        if (distance2 > max_distance_ || distance2 < min_distance_ ||
            (angle_flag_ && (arg_horiz < start_angle_ || arg_horiz > end_angle_)) ||
            (!angle_flag_ && (arg_horiz > end_angle_ && arg_horiz < start_angle_)))  // invalid distance
        {
          point.x = NAN;
          point.y = NAN;
          point.z = NAN;
          point.intensity = 0;
          pointcloud->at(this->block_num, dsr) = point;
        }
        else
        {
          // If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
          point.x = distance2 * this->cos_lookup_table_[arg_vert] * this->cos_lookup_table_[arg_horiz] +
                    Rx_ * this->cos_lookup_table_[arg_horiz_orginal];
          point.y = -distance2 * this->cos_lookup_table_[arg_vert] * this->sin_lookup_table_[arg_horiz] -
                    Rx_ * this->sin_lookup_table_[arg_horiz_orginal];
          point.z = distance2 * this->sin_lookup_table_[arg_vert] + Rz_;
          point.intensity = intensity;
          pointcloud->at(this->block_num, dsr) = point;
        }
      }
    }
    else
    {
      // Estimate the type of packet
      union two_bytes tmp_flag;
      tmp_flag.bytes[1] = raw->blocks[block].data[0];
      tmp_flag.bytes[0] = raw->blocks[block].data[1];
      int ABflag = isABPacket(tmp_flag.uint);

      int k = 0;
      int index;
      for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++, k += RAW_SCAN_SIZE)  // 16   3
      {
        if (ABflag == 1 && dsr < 16)
        {
          index = k + 48;
        }
        else if (ABflag == 1 && dsr >= 16)
        {
          index = k - 48;
        }
        else
        {
          index = k;
        }

        int dsr_temp;
        if (dsr >= 16)
        {
          dsr_temp = dsr - 16;
        }
        else
        {
          dsr_temp = dsr;
        }
        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr_temp * RS32_DSR_TOFFSET)) / RS32_BLOCK_TDURATION);
        azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);

        union two_bytes tmp;
        tmp.bytes[1] = raw->blocks[block].data[index];
        tmp.bytes[0] = raw->blocks[block].data[index + 1];
        int ab_flag_in_block = isABPacket(tmp.uint);
        int distance = tmp.uint - ab_flag_in_block * 32768;

        // read intensity
        intensity = (float)raw->blocks[block].data[index + 2];
        intensity = calibrateIntensity(intensity, dsr, distance);

        float distance2 = pixelToDistance(distance, dsr);
        distance2 = distance2 * DISTANCE_RESOLUTION;

        int arg_horiz_orginal = (int)azimuth_corrected_f % 36000;
        int arg_horiz = azimuth_corrected;
        int arg_vert = ((VERT_ANGLE[dsr]) % 36000 + 36000) % 36000;

        pcl::PointXYZI point;

        if (distance2 > max_distance_ || distance2 < min_distance_ ||
            (angle_flag_ && (arg_horiz < start_angle_ || arg_horiz > end_angle_)) ||
            (!angle_flag_ && (arg_horiz > end_angle_ && arg_horiz < start_angle_)))  // invalid distance
        {
          point.x = NAN;
          point.y = NAN;
          point.z = NAN;
          point.intensity = 0;
          pointcloud->at(this->block_num, dsr) = point;
        }
        else
        {
          // If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
          point.x = distance2 * this->cos_lookup_table_[arg_vert] * this->cos_lookup_table_[arg_horiz] +
                    Rx_ * this->cos_lookup_table_[arg_horiz_orginal];
          point.y = -distance2 * this->cos_lookup_table_[arg_vert] * this->sin_lookup_table_[arg_horiz] -
                    Rx_ * this->sin_lookup_table_[arg_horiz_orginal];
          point.z = distance2 * this->sin_lookup_table_[arg_vert] + Rz_;
          point.intensity = intensity;
          pointcloud->at(this->block_num, dsr) = point;
        }
      }
    }
  }
}

}  // namespace rslidar_rawdata
