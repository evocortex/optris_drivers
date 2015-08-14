/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2015
 *  Technische Hochschule Nürnberg Georg Simon Ohm
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Nuremberg Institute of Technology
 *     Georg Simon Ohm nor the authors names may be used to endorse
 *     or promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Stefan May
 *********************************************************************/

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/TimeReference.h>
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "optris_drivers/AutoFlag.h"
#include "optris_drivers/SwitchTemperatureRange.h"
#include <optris_drivers/Temperature.h>

#include "libirimager/IRImager.h"
#include "libirimager/ImageBuilder.h"

#include <sys/stat.h>
using namespace std;

sensor_msgs::Image _thermal_image;
sensor_msgs::Image _visible_image;
sensor_msgs::TimeReference _optris_timer;
optris_drivers::Temperature _internal_temperature;

image_transport::Publisher* _thermal_pub = NULL;
image_transport::Publisher* _visible_pub = NULL;

ros::Publisher _timer_pub;
ros::Publisher _temp_pub;

unsigned int _img_cnt = 0;
optris::IRImager* _imager;

/**
 * Callback method from image processing library (called at configured frame rate in xml file)
 * @param[in] image thermal image in unsigned short format, i.e., float temperature = ((float)image[i] -1000.f)/10.f)
 * @param[in] w image width
 * @param[in] h image height
 * @param[in] timestamp the frame's timestamp
 * @param[in] arg user argument passed to process method
 */
void onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, long long timestamp, void* arg)
{
  memcpy(&_thermal_image.data[0], image, w * h * sizeof(*image));

  _thermal_image.header.seq = ++_img_cnt;
  _thermal_image.header.stamp = ros::Time::now();
  _thermal_pub->publish(_thermal_image);

  _optris_timer.header.seq = _thermal_image.header.seq;
  _optris_timer.header.stamp = _thermal_image.header.stamp;
  _optris_timer.time_ref.fromNSec(timestamp);

  _internal_temperature.header.seq=_thermal_image.header.seq;
  _internal_temperature.header.stamp = _thermal_image.header.stamp;

  _internal_temperature.temperature_flag = _imager->getTempFlag();
  _internal_temperature.temperature_box = _imager->getTempBox();
  _internal_temperature.temperature_chip = _imager->getTempChip();

  _timer_pub.publish(_optris_timer);
  _temp_pub.publish(_internal_temperature);
}

/**
 * Callback method from image processing library (called at configured frame rate in xml file)
 * @param[in] image RGB image, if BISPECTRAL technology is available
 * @param[in] w image width
 * @param[in] h image height
 * @param[in] timestamp the frame's timestamp
 * @param[in] arg user argument passed to process method
 */
void onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, long long timestamp, void* arg)
{
  if(_visible_pub->getNumSubscribers()==0) return;

  memcpy(&_visible_image.data[0], image, 2 * w * h * sizeof(*image));

  _visible_image.header.seq   = _img_cnt;
  _visible_image.header.stamp = ros::Time::now();
  _visible_pub->publish(_visible_image);
}

bool onAutoFlag(optris_drivers::AutoFlag::Request &req, optris_drivers::AutoFlag::Response &res)
{
  _imager->setAutoFlag(req.autoFlag);
  res.isAutoFlagActive = _imager->getAutoFlag();
  return true;
}

bool onForceFlag(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  _imager->forceFlagEvent();
  return true;
}

bool onSwitchTemperatureRange(optris_drivers::SwitchTemperatureRange::Request &req, optris_drivers::SwitchTemperatureRange::Response &res)
{
  bool validParam = true;
  switch(req.temperatureRange)
  {
  case 0:
    _imager->setTempRange(optris::TM20_100);
    break;
  case 1:
    _imager->setTempRange(optris::T0_250);
    break;
  case 2:
    _imager->setTempRange(optris::T150_900);
    break;
  default:
    std::cerr << "Wrong temperature range parameter passed, valid = [0, 1, 2]" << endl;
    validParam = false;
    break;
  }
  if(validParam)
  {
    _imager->forceFlagEvent(1000.f);
    res.success = true;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optris_imager_node");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");

  std::string xmlConfig = "";
  n_.getParam("xmlConfig", xmlConfig);

  // A specific configuration file for each imager device is needed (cf. config directory)
  struct stat s;
  if(stat(xmlConfig.c_str(), &s) != 0)
  {
    std::cerr << "usage: rosrun <package> <node> _xmlConfig:=<xmlConfig>" << std::endl;
    std::cerr << " verify that <xmlConfig> exists" << std::endl;
    return -1;
  }

  ros::NodeHandle n;

  _imager = new optris::IRImager(xmlConfig.c_str());

  unsigned char* bufferRaw = new unsigned char[_imager->getRawBufferSize()];

  image_transport::ImageTransport it(n);

  _imager->setFrameCallback(onThermalFrame);

  image_transport::Publisher tpub = it.advertise("thermal_image", 1);
  _thermal_pub = &tpub;

  _thermal_image.header.frame_id = "thermal_image";
  _thermal_image.height          = _imager->getHeight();
  _thermal_image.width           = _imager->getWidth();
  _thermal_image.encoding        = "mono16";
  _thermal_image.step            = _thermal_image.width * 2;
  _thermal_image.data.resize(_thermal_image.height * _thermal_image.step);


  image_transport::Publisher vpub;
  if(_imager->hasBispectralTechnology())
  {
    _imager->setVisibleFrameCallback(onVisibleFrame);

    vpub = it.advertise("visible_image", 1);
    _visible_pub = &vpub;

    _visible_image.header.frame_id = "visible_image";
    _visible_image.height          = _imager->getVisibleHeight();
    _visible_image.width           = _imager->getVisibleWidth();
    _visible_image.encoding        = "yuv422";
    _visible_image.step            = _visible_image.width * 2;
    _visible_image.data.resize(_visible_image.height * _visible_image.step);
  }


  // advertise the camera internal timer
   _timer_pub= n.advertise<sensor_msgs::TimeReference>("optris_timer", 1 );
  _optris_timer.header.frame_id=_thermal_image.header.frame_id;


  ros::ServiceServer sAuto  = n_.advertiseService("auto_flag",  onAutoFlag);
  ros::ServiceServer sForce = n_.advertiseService("force_flag", onForceFlag);
  ros::ServiceServer sTemp = n_.advertiseService("switch_temperature_range", onSwitchTemperatureRange);

  // advertise all of the camera's temperatures in a single custom message
  _temp_pub = n.advertise <optris_drivers::Temperature> ("internal_temperature", 1);
  _internal_temperature.header.frame_id=_thermal_image.header.frame_id;


  _imager->startStreaming();

  // loop over acquire-process-release-publish steps
  // Images are published in raw temperature format (unsigned short, see onFrame callback for details)
  ros::Rate loop_rate(_imager->getMaxFramerate());
  while(ros::ok())
  {
    if(_imager->getFrame(bufferRaw)==optris::IRIMAGER_SUCCESS)
    {
      _imager->process(bufferRaw);
      _imager->releaseFrame();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();

  delete[] bufferRaw;
  delete _imager;

  return 0;
}
