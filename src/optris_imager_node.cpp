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

#include "PIImager.h"
#include "ImageBuilder.h"

#include <sys/stat.h>

sensor_msgs::Image _thermal_image;
ros::Publisher     _img_pub;
unsigned int       _img_cnt = 0;
/**
 * Callback method from image processing library, called with configured frame rate, see xml file
 * @param[in] image thermal image in unsigned short format, i.e., float temperature = ((float)image[i] -1000.f)/10.f)
 * @param[in] w image width
 * @param[in] h image height
 */
void onFrame(unsigned short* image, unsigned int w, unsigned int h)
{
	memcpy(&_thermal_image.data[0], image, w*h*sizeof(*image));

	_thermal_image.header.seq   = _img_cnt++;
	_thermal_image.header.stamp = ros::Time::now();

	_img_pub.publish(_thermal_image);
	ros::spinOnce();
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "optris_imager_node");

	// private node handle to support command line parameters for rosrun
	ros::NodeHandle n_("~");

	std::string xmlConfig = "";
	n_.getParam("xmlConfig", xmlConfig);

	// A specific configuration file for each imager device is needed (cf. config directory)
	struct stat s;
	if(stat(xmlConfig.c_str(), &s) != 0)
	{
		std::cerr << "usage: rosrun <package> <node> _xmlConfig:=<xmlConfig>" << std::endl;
		std::cerr << " verify that <xmlConfig> is existent" << std::endl;
		return -1;
	}

	ros::NodeHandle n;

	optris::PIImager imager(xmlConfig.c_str());

	_img_pub = n.advertise<sensor_msgs::Image>("thermal_image" , 1);

	unsigned char* bufferRaw = new unsigned char[imager.getRawBufferSize()];

	imager.setFrameCallback(onFrame);

	_thermal_image.header.frame_id = "thermal_image";
	_thermal_image.height          = imager.getHeight();
	_thermal_image.width 	       = imager.getWidth();
	_thermal_image.encoding        = "mono16";
	_thermal_image.step		       = _thermal_image.width*2;
	_thermal_image.data.resize(_thermal_image.height*_thermal_image.step);

	imager.startStreaming();

	// loop over acquire-process-release-publish steps
	// Images are published in raw format (unsigned short, see onFrame callback for details)
	ros::Rate loop_rate(imager.getMaxFramerate());
	while (ros::ok())
	{
		imager.getFrame(bufferRaw);
		imager.process(bufferRaw);
		imager.releaseFrame();
		loop_rate.sleep();
	}

	delete [] bufferRaw;

	return 0;
}
