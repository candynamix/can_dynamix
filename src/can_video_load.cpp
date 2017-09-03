
// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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
*   * Neither the name of the Kei Okada nor the names of its
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
*********************************************************************/


//http://wiki.ros.org/image_transport/Tutorials/PublishingImages

/*
   can_dynamix team 
   JinYoung Kim

   2017-09-03
*/



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <stdio.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
///  if(argv[1] == NULL) return 1;

  ros::init(argc, argv, "video_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("can_dynamix/video", 1);

  // Convert the passed as command line parameter index for the video device to an integer
///  std::istringstream video_sourceCmd("2.mp4");
///  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  // Check if it is indeed a number
///  if(!(video_sourceCmd >> video_source)) return 1;
///      ROS_INFO("2 ");
///  cv::VideoCapture cap(video_source);
  cv::VideoCapture cap(0);

  // Check if video device can be opened with the given index
  if(!cap.isOpened()) { 
      ROS_ERROR("unable open video file ");
	return 1; 
  }


  cv::Mat frame(320,240,CV_8UC3);
  sensor_msgs::ImagePtr msg;

/* /---

double fps = 5;
int fourcc = CV_FOURCC('X','V','I','D'); // codec (opencv3.0이하)
bool isColor = true;

cv::VideoWriter *video = new VideoWriter;
if(!video->open("result.avi", fourcc, fps, cv::Size(frame.rows, frame.cols), isColor)){
//cv::delete video;
//return;
}

*/


  ros::Rate loop_rate(5);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
//      ROS_INFO("video out ");
 //     cv::waitKey(1);
     
    }
//    ROS_INFO("--- ");
    ros::spinOnce();
    loop_rate.sleep();
  }
}
