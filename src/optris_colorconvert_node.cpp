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

#include "libirimager/ImageBuilder.h"

unsigned char*                    _bufferThermal = NULL;
unsigned char*                    _bufferVisible = NULL;
image_transport::CameraPublisher* _pubThermal;
image_transport::Publisher*       _pubVisible;
unsigned int                      _frame = 0;


std::string _thermalimage_topic, _visibleimage_topic;
optris::ImageBuilder              _iBuilder;
optris::EnumOptrisColoringPalette _palette;

void onThermalDataReceive(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr & cam_info)
{
   // check for any subscribers to save computation time
  if(_pubThermal->getNumSubscribers() == 0)
     return;

  unsigned short* data = (unsigned short*)&image->data[0];
  _iBuilder.setData(image->width, image->height, data);

  if(_bufferThermal==NULL)
    _bufferThermal = new unsigned char[image->width * image->height * 3];

  _iBuilder.convertTemperatureToPaletteImage(_bufferThermal, true);

  sensor_msgs::Image img;

  img.header.frame_id = "thermal_image_view";
  img.height 	        = image->height;
  img.width 	        = image->width;
  img.encoding        = "rgb8";
  img.step            = image->width*3;
  img.data.resize(img.height*img.step);
  img.header.seq      = ++_frame;
  img.header.stamp    = image->header.stamp;

  for(unsigned int i=0; i<image->width*image->height*3; i++) {
    img.data[i] = _bufferThermal[i];
  }

  _pubThermal->publish(img,*cam_info);
}

void onVisibleDataReceive(const sensor_msgs::ImageConstPtr& image)
{
  // check for any subscribers to save computation time
  if(_pubVisible->getNumSubscribers() == 0)
     return;

  if(_bufferVisible==NULL)
    _bufferVisible = new unsigned char[image->width * image->height * 3];

  const unsigned char* data = &image->data[0];
  _iBuilder.yuv422torgb24(data, _bufferVisible, image->width, image->height);

  sensor_msgs::Image img;
  img.header.frame_id = "visible_image_view";
  img.height          = image->height;
  img.width           = image->width;
  img.encoding        = "rgb8";
  img.step            = image->width*3;
  img.data.resize(img.height*img.step);

  img.header.seq      = _frame;
  img.header.stamp    = image->header.stamp;

  for(unsigned int i=0; i<image->width*image->height*3; i++) {
    img.data[i] = _bufferVisible[i];
  }

  _pubVisible->publish(img);
}

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "optris_colorconvert_node");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");

  int palette = 6;
  n_.getParam("palette", palette);
  _palette = (optris::EnumOptrisColoringPalette) palette;

  optris::EnumOptrisPaletteScalingMethod scalingMethod = optris::eMinMax;
  int sm;
  n_.getParam("paletteScaling", sm);
  if(sm>=1 && sm <=4) scalingMethod = (optris::EnumOptrisPaletteScalingMethod) sm;

  _iBuilder.setPaletteScalingMethod(scalingMethod);
  _iBuilder.setPalette(_palette);

  double tMin     = 20.;
  double tMax     = 40.;
  double looprate = 30.;

  n_.getParam("temperatureMin", tMin);
  n_.getParam("temperatureMax", tMax);
  n_.getParam("looprate",       looprate);

  _iBuilder.setManualTemperatureRange((float)tMin, (float)tMax);

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  n_.param<std::string>("thermal_topic", _thermalimage_topic, "thermal_image");
  n_.param<std::string>("visible_topic", _visibleimage_topic, "visible_image");

  image_transport::Subscriber subVisible = it.subscribe(_visibleimage_topic, 1, onVisibleDataReceive);
  image_transport::CameraSubscriber subThermal= it.subscribeCamera(_thermalimage_topic, 1, onThermalDataReceive);

  image_transport::CameraPublisher pubt = it.advertiseCamera("/thermal_image_view/image_raw", 1);
  image_transport::Publisher pubv = it.advertise("visible_image_view", 1);

  _pubThermal = &pubt;
  _pubVisible = &pubv;

  // set to png compression
  std::string key;
  if(ros::param::search("thermal_image/compressed/format", key))
  {
     ros::param::set(key, "png");
  }
  if(ros::param::search("thermal_image/compressed/png_level", key))
  {
     ros::param::set(key, 9);
  }

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(looprate);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if(_bufferThermal)	 delete [] _bufferThermal;
  if(_bufferVisible)  delete [] _bufferVisible;
}
