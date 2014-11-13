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
#include <dynamic_reconfigure/server.h>

#include <optris_drivers/ThresholdConfig.h>

#include "libirimager/ImageBuilder.h"

unsigned char*                    _bufferThermal = NULL;
unsigned char*                    _bufferVisible = NULL;
image_transport::Publisher*       _pubThermal;
image_transport::Publisher*       _pubVisible;
unsigned int                      _frame = 0;

optris::ImageBuilder              _iBuilder;
optris::EnumOptrisColoringPalette _palette;

double _threshold = 30.0;
bool   _invert    = false;

void onThermalDataReceive(const sensor_msgs::ImageConstPtr& image)
{
  unsigned short* data = (unsigned short*)&image->data[0];

  sensor_msgs::Image img;
  img.header.frame_id = "thermal_image_view";
  img.height 	       = image->height;
  img.width 	       = image->width;
  img.encoding        = "mono8";
  img.step            = image->width;
  img.data.resize(img.height*img.step);
  img.header.seq      = ++_frame;
  img.header.stamp    = ros::Time::now();


  for(unsigned int i=0; i<image->width*image->height; i++)
  {
     const double temp = (float(data[i]) -1000.0f)/10.0f;

     if(!_invert) {
        if(temp > _threshold) img.data[i] = 0xff;
        else                  img.data[i] = 0x00;
     }
     else {
        if(temp > _threshold) img.data[i] = 0x00;
        else                  img.data[i] = 0xff;
     }

  }

  _pubThermal->publish(img);
}

void callback(optris_drivers::ThresholdConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %d",
            config.threshold, config.invert);

  _threshold = config.threshold;
  _invert    = config.invert;
}


int main (int argc, char* argv[])
{
  ros::init (argc, argv, "optris_binary_image_node");

  dynamic_reconfigure::Server<optris_drivers::ThresholdConfig> server;
  dynamic_reconfigure::Server<optris_drivers::ThresholdConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

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

  double tMax = 40.;
  n_.getParam("threshold", _threshold);
//  _iBuilder.setManualTemperatureRange((float)tMin, (float)tMax);

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("thermal_image",  1, onThermalDataReceive);
  image_transport::Publisher pubt        = it.advertise("thermal_binary", 1);
  _pubThermal = &pubt;

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if(_bufferThermal)	 delete [] _bufferThermal;
  if(_bufferVisible)  delete [] _bufferVisible;
}
