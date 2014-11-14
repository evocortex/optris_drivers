/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2014
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
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "optris_drivers/AutoFlag.h"

#include "libirimager/IRImager.h"
#include "libirimager/ImageBuilder.h"

#include <sys/stat.h>
using namespace std;

sensor_msgs::Image _thermal_image;
sensor_msgs::Image _visible_image;
std_msgs::Float32  _flag_temperature;
std_msgs::Float32  _box_temperature;
std_msgs::Float32  _chip_temperature;

image_transport::Publisher* _thermal_pub = NULL;
image_transport::Publisher* _visible_pub = NULL;

ros::Publisher _flag_pub;
ros::Publisher _box_pub;
ros::Publisher _chip_pub;

unsigned int _img_cnt = 0;
optris::IRImager* _imager;

/**
 * Callback method from image processing library (called at configured frame rate in xml file)
 * @param[in] image thermal image in unsigned short format, i.e., float temperature = ((float)image[i] -1000.f)/10.f)
 * @param[in] w image width
 * @param[in] h image height
 */
void onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, long long timestamp)
{
  memcpy(&_thermal_image.data[0], image, w * h * sizeof(*image));

  _thermal_image.header.seq = ++_img_cnt;
  _thermal_image.header.stamp = ros::Time::now();
  _thermal_pub->publish(_thermal_image);

  _flag_temperature.data = _imager->getTempFlag();
  _box_temperature.data  = _imager->getTempBox();
  _chip_temperature.data = _imager->getTempChip();
  _flag_pub.publish(_flag_temperature);
  _box_pub.publish(_box_temperature);
  _chip_pub.publish(_chip_temperature);
}

void onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h)
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
  return true;
}

bool onForceFlag(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  _imager->forceFlagEvent();
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

  ros::ServiceServer sAuto  = n_.advertiseService("auto_flag",  onAutoFlag);
  ros::ServiceServer sForce = n_.advertiseService("force_flag", onForceFlag);

  _flag_pub = n.advertise <std_msgs::Float32> ("temperature_flag", 1);
  _box_pub  = n.advertise <std_msgs::Float32> ("temperature_box", 1);
  _chip_pub = n.advertise <std_msgs::Float32> ("temperature_chip", 1);


  _imager->startStreaming();

  // loop over acquire-process-release-publish steps
  // Images are published in raw temperature format (unsigned short, see onFrame callback for details)
  ros::Rate loop_rate(_imager->getMaxFramerate());
  while(ros::ok())
  {
    _imager->getFrame(bufferRaw);
    _imager->process(bufferRaw);
    _imager->releaseFrame();
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();

  delete[] bufferRaw;
  delete _imager;

  return 0;
}
