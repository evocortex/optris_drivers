/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012,
 *  Georg Simon Ohm University of Applied Sciences Nuremberg
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
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
#include "sensor_msgs/Image.h"

#include "ImageBuilder.h"

unsigned char* _buffer = NULL;
ros::Publisher _pub;
unsigned int _frame = 0;

optris::ImageBuilder _iBuilder;
optris::EnumOptrisColoringPalette _palette;

void onDataReceive(const sensor_msgs::Image& image)
{
	if(_buffer==NULL)
        _buffer = new unsigned char[image.width * image.height * 3];

	_iBuilder.setSize(image.width, image.height, false);

	const unsigned char* data = &image.data[0];
	_iBuilder.convertTemperatureToPaletteImage((unsigned short*)data, _buffer);

	sensor_msgs::Image img;
	img.header.frame_id = "thermal_image_view";
	img.height 	        = image.height;
	img.width 	        = image.width;
	img.encoding        = "rgb8";
	img.step		    = image.width*3;
	img.data.resize(img.height*img.step);

	img.header.seq      = _frame++;
	img.header.stamp    = ros::Time::now();

	for(unsigned int i=0; i<image.width*image.height*3; i++)
	{
        img.data[i] = _buffer[i];
	}

	_pub.publish(img);
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
	ros::Subscriber sub = n.subscribe("thermal_image", 1, onDataReceive);
	_pub                = n.advertise<sensor_msgs::Image>("thermal_image_view" , 1);

	// specify loop rate a meaningful value according to your publisher configuration
	ros::Rate loop_rate(30);
	while (ros::ok())
	{
        ros::spinOnce();
        loop_rate.sleep();
	}

	if(_buffer)	delete [] _buffer;
}
