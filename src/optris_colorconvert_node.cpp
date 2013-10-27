/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012/2013
 *  Nuremberg Institute of Technology Georg Simon Ohm
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

#include "ImageBuilder.h"

unsigned char*                    _bufferThermal = NULL;
unsigned char*                    _bufferVisible = NULL;
image_transport::Publisher*       _pubThermal;
image_transport::Publisher*       _pubVisible;
unsigned int                      _frame = 0;

optris::ImageBuilder              _iBuilder;
optris::EnumOptrisColoringPalette _palette;

void onThermalDataReceive(const sensor_msgs::ImageConstPtr& image)
{
  if(_bufferThermal==NULL)
    _bufferThermal = new unsigned char[image->width * image->height * 3];

  unsigned short* data = (unsigned short*)&image->data[0];

  _iBuilder.setData(image->width, image->height, data);

  _iBuilder.convertTemperatureToPaletteImage(_bufferThermal);

  sensor_msgs::Image img;
  img.header.frame_id = "thermal_image_view";
  img.height 	        = image->height;
  img.width 	        = image->width;
  img.encoding        = "rgb8";
  img.step            = image->width*3;
  img.data.resize(img.height*img.step);

  img.header.seq      = ++_frame;
  img.header.stamp    = ros::Time::now();

  for(unsigned int i=0; i<image->width*image->height*3; i++)
  {
    img.data[i] = _bufferThermal[i];
  }

  _pubThermal->publish(img);
}

void onVisibleDataReceive(const sensor_msgs::ImageConstPtr& image)
{
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
  img.header.stamp    = ros::Time::now();

  for(unsigned int i=0; i<image->width*image->height*3; i++)
  {
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

  double tMin = 20.;
  double tMax = 40.;
  n_.getParam("temperatureMin", tMin);
  n_.getParam("temperatureMax", tMax);
  _iBuilder.setManualTemperatureRange((float)tMin, (float)tMax);

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("thermal_image", 1, onThermalDataReceive);
  image_transport::Subscriber subVisible = it.subscribe("visible_image", 1, onVisibleDataReceive);

  image_transport::Publisher pubt = it.advertise("thermal_image_view", 1);
  image_transport::Publisher pubv = it.advertise("visible_image_view", 1);
  _pubThermal = &pubt;
  _pubVisible = &pubv;

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if(_bufferThermal)	delete [] _bufferThermal;
  if(_bufferVisible)  delete [] _bufferVisible;
}
