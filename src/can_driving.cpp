

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
//https://github.com/robotpilot
/*
   can_dynamix team 
   JinYoung Kim

   2017-09-03
*/

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <dynamic_reconfigure/server.h>


namespace can_dynamix {

// static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
    geometry_msgs::TwistPtr cmd; 
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher vel_pub_   = nh_.advertise<geometry_msgs::Twist>("can_dynamix/cmd_vel", 100); 
  
  bool debug_view_;

public:
  ImageConverter()
    : 
	cmd(new geometry_msgs::Twist()),
	it_(nh_)
  {
    image_sub_ = it_.subscribe("/cv_camera/video", 1,  &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/can_dynamix/video_out", 1);
//    vel_pub_   = it_.advertise<geometry_msgs::Twist>("/can_dyanamix/cmd_vel",1);

//    ros::NodeHandle pnh_("~");
//    pnh_.param("debug_view", debug_view_, false);
  }

  ~ImageConverter()
  {
//    if( debug_view_) {
//      cv::destroyWindow(OPENCV_WINDOW);
//    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat in_image, jout_image ;
	 float angle;
    try
    {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  

      if (in_image.channels() > 1) {
        cv::cvtColor( in_image, jout_image, cv::COLOR_BGR2GRAY );
        /// Apply Canny edge detector
        GaussianBlur(jout_image, jout_image, cv::Size(9,9), 1.5); 
        cv::Canny( jout_image, jout_image, 50, 200, 3 );
      }
      else {
        /// Check whether input gray image is filtered such that canny, sobel ...etc
        bool is_filtered = true;
        for(int y=0; y < in_image.rows; ++y) {
          for(int x=0; x < in_image.cols; ++x) {
            if(!(in_image.at<unsigned char>(y, x) == 0
                 || in_image.at<unsigned char>(y, x) == 255)) {
              is_filtered = false;
              break;
            }
            if(!is_filtered) {
              break;
            }
          }
        }

        if(!is_filtered) {
          cv::Canny( jout_image, jout_image, 50, 200, 3 );
        }
      }


    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

     std::vector<cv::Vec4i> lines;
     cv::HoughLinesP(jout_image, lines,1,CV_PI/180, 30, 30, 3);
     cv::cvtColor(jout_image, jout_image, CV_GRAY2BGR);


  	cmd->linear.x = 3.1234;
  	cmd->linear.y = 0.0;
  	cmd->linear.z = 0.0;
  	cmd->angular.x = 0.0;
  	cmd->angular.y = 0.0;
  	cmd->angular.z = 0.0;
     
	for(int i=0; i< lines.size(); i++)
	{
	   cv::Vec4i L = lines[i];
           angle = (atan2(L[1] - L[3], L[0] - L[2]))*(180 / CV_PI);
	   if(angle < 160 && angle > 90)
	   cv::line(jout_image, cv::Point(L[0],L[1]), cv::Point(L[2],L[3]), cv::Scalar(0,0,255),2,CV_AA);
	   if(angle > -160 && angle < -90)
	   cv::line(jout_image, cv::Point(L[0],L[1]), cv::Point(L[2],L[3]), cv::Scalar(255,0,0),2,CV_AA);
	}

  	   cmd->angular.z = angle;	
        ROS_INFO("---");

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 110 && cv_ptr->image.cols > 110) 
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 100, CV_RGB(255,0,0));

      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", jout_image).toImageMsg();

    image_pub_.publish(out_img);
    vel_pub_.publish(cmd);
  }
};

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "can_image"); 



  while (ros::ok())
  { 
      can_dynamix::ImageConverter  imageCon ;
      ros::spin();

  }

  return 0;
}
