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

unsigned short* _thermal_image;

/**
 * Callback method from image processing library
 * @param[in] image thermal image in unsigned short format, i.e., float temperature = ((float)image[i] -1000.f)/10.f)
 * @param[in] w image width
 * @param[in] h image height
 */
void onFrame(unsigned short* image, unsigned int w, unsigned int h)
{
	memcpy(_thermal_image, image, w*h*sizeof(*_thermal_image));
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "optris_imager_node");
	ros::NodeHandle n;

	std::string xmlConfig = "";
	n.getParam("xmlConfig", xmlConfig);

	std::cout << xmlConfig << std::endl;

	// A specific configuration file for each imager is needed (cf. config directory)
	struct stat s;
	if(stat(xmlConfig.c_str(), &s) != 0)
	{
		std::cerr << "usage: rosrun <package> <node> _xmlConfig:=<xmlConfig>" << std::endl;
		return -1;
	}

	optris::PIImager imager(xmlConfig.c_str());

	ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("thermal_image" , 1);
	ros::Rate loop_rate(imager.getFramerate());

	unsigned char* bufferRaw = new unsigned char[imager.getRawBufferSize()];
	_thermal_image           = new unsigned short[imager.getWidth() * imager.getHeight()];

	imager.setFrameCallback(onFrame);

	sensor_msgs::Image img;
	img.header.frame_id = "thermal_image";
	img.height          = imager.getHeight();
	img.width 	        = imager.getWidth();
	img.encoding        = "mono16";
	img.step		    = img.width*2;
	img.data.resize(img.height*img.step);	

	imager.startStreaming();

	// loop over acquire-process-release-publish steps
	// Images are published in raw format (unsigned short, see onFrame callback for details)
	unsigned int i = 0;
	while (ros::ok())
	{
		imager.getFrame(bufferRaw);
		imager.process(bufferRaw);
		imager.releaseFrame();

		ros::spinOnce();
		img.header.seq   = i++;
		img.header.stamp = ros::Time::now();

		memcpy(&img.data[0], _thermal_image, img.height*img.width*sizeof(*_thermal_image));

		img_pub.publish(img);
	}

	delete [] bufferRaw;
	delete [] _thermal_image;
}
