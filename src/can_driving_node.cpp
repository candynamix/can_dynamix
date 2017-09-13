

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
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <dynamic_reconfigure/server.h>


#define TestVideo2

namespace can_dynamix {

// static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  geometry_msgs::TwistPtr cmd; 
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher  image_pub_;
  image_transport::Publisher  image_pub2_;
  ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("can_dynamix/cmd_vel", 100); 
  
  bool debug_view_;

public:
  ImageConverter()
    : 
	cmd(new geometry_msgs::Twist()),
	it_(nh_)
  {
    image_sub_ = it_.subscribe("/can_dynamix/video", 1,  &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/can_dynamix/video_out1", 1);
#ifdef TestVideo2
    image_pub2_ = it_.advertise("/can_dynamix/video_out2", 1);
#endif
  }

  void videoCb(const sensor_msgs::ImageConstPtr& msg)
  {

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat in_image, jout_image ;
#ifdef TestVideo2
        cv::Mat src_gray;	
	cv::Mat yellow_hue_image;
	cv::Mat lower_yellow_hue_range;
	cv::Mat upper_yellow_hue_range;
#endif
	float angle, l_angle, r_angle;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  
      	if (in_image.channels() > 1) {

        cv::cvtColor( in_image, jout_image, cv::COLOR_BGR2GRAY );
        /// Apply Canny edge detector

        GaussianBlur(jout_image, jout_image, cv::Size(9,9), 1.5); 
        cv::Canny( jout_image, jout_image, 50, 200, 3 );

#ifdef TestVideo2
        cv::cvtColor( in_image, src_gray, cv::COLOR_BGR2HSV );
	// Threshold the HSV image, keep only the red pixels // 0 - 10
	cv::inRange(src_gray, cv::Scalar(30, 150, 100), cv::Scalar(60, 255, 255), lower_yellow_hue_range);
	cv::inRange(src_gray, cv::Scalar(15, 100, 100), cv::Scalar(90, 255, 255), upper_yellow_hue_range);
	cv::addWeighted(lower_yellow_hue_range, 1.0, upper_yellow_hue_range, 1.0, 0.0, yellow_hue_image);
        cv::Canny( yellow_hue_image, yellow_hue_image, 50, 200, 3 );
#endif
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
     //  End of try
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


     cv::Point pt1, pt2, pt3, pt4;
     float l_angle3, r_angle3;

 
     std::vector<cv::Vec2f> lines0;
     cv::HoughLines(jout_image, lines0, 1, CV_PI / 180, 50, 0, 0);

     std::vector<cv::Vec4i> lines;
     cv::HoughLinesP(jout_image, lines,1,CV_PI/180, 30, 10, 10);
     cv::cvtColor(jout_image, jout_image, CV_GRAY2BGR);
#ifdef TestVideo2
     std::vector<cv::Vec4i> lines2;
     cv::HoughLinesP(yellow_hue_image, lines2,1,CV_PI/180, 30, 10, 10);
     cv::cvtColor(yellow_hue_image, yellow_hue_image, CV_GRAY2BGR);
#endif

//     cv::Rect roi(100,100,100,100); //x,y좌표 100,100에 w,h가 100x100인 roi를 예로 들죠

  	cmd->linear.x = 3.1234;
  	cmd->linear.y = 0.0;
  	cmd->linear.z = 0.0;
  	cmd->angular.x = 0.0;
  	cmd->angular.y = 0.0;
  	cmd->angular.z = 0.0;

///---------------------------------------- video 1 ------------------------------------------------

	for(int i=0; i< lines.size(); i++)
	{
	   cv::Vec4i L = lines[i];
           angle = (atan2(L[1] - L[3], L[0] - L[2]))*(180 / CV_PI);
//	   if(((angle < 160 && angle > 98) || (angle < 90 && angle > 10 )) && ( cvRound(L[1]) >= 200)  && ( cvRound(L[3]) <= 260 )) {
	   if(((angle < 160 && angle > 98) || (angle < 85 && angle > 10 ))  && ( cvRound(L[1]) >= 200) ) {
	      	cv::line(jout_image, cv::Point(L[0],L[1]), cv::Point(L[2],L[3]), cv::Scalar(0,0,255),2,CV_AA);
	      	l_angle = angle;
		#ifdef test
              	std::stringstream l_lineText; 
              	l_lineText << "left ";
              	l_lineText << i;
 	      	cv::putText(jout_image,l_lineText.str(), cv::Point(L[0],L[1]),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(255,255,255),1);
              	ROS_INFO("Left= %i, %f  (%i,%i)-(%i,%i) ", i, angle,L[0],L[1], L[2],L[3]);
		#endif
	   }
	   if(((angle > -160 && angle < -98) || (angle > -85  && angle < -10)) && ( cvRound(L[1]) >= 200 ) ) {
              	std::stringstream r_lineText; 
              	r_lineText << "right ";
              	r_lineText << i;
	      	cv::line(jout_image, cv::Point(L[0],L[1]), cv::Point(L[2],L[3]), cv::Scalar(255,0,0),2,CV_AA);
 	      	cv::putText(jout_image,r_lineText.str(), cv::Point(L[0],L[1]),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(255,255,255),1);
              	ROS_INFO("Right= %i, %f  (%i,%i)-(%i,%i) ", i, angle,L[0],L[1], L[2],L[3]);
	      	r_angle = angle;
	   }
	}
#ifdef TestVideo2
///---------------------------------------- video 2 ------------------------------------------------
	for(int i=0; i< lines2.size(); i++)
	{
	   cv::Vec4i L2 = lines2[i];
           angle = (atan2(L2[1] - L2[3], L2[0] - L2[2]))*(180 / CV_PI);
	   if((angle < 160 && angle > 90)&&(cvRound(L2[1]) > jout_image.rows*6/7 )&&(cvRound(L2[1]) < jout_image.rows*7/7 )) {
	   cv::line(yellow_hue_image, cv::Point(L2[0],L2[1]), cv::Point(L2[2],L2[3]), cv::Scalar(0,0,255),2,CV_AA);
//	   l_angle = angle; 
 	   cv::putText(yellow_hue_image,"Left_lines", cv::Point(L2[0],L2[1]),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(255,255,255),1);
	   }	
	   if((angle > -160 && angle < -90)&&(cvRound(L2[1]) > jout_image.rows*6/7)&&(cvRound(L2[1]) < jout_image.rows*7/7 )) {
 	   cv::putText(yellow_hue_image,"Right_lines", cv::Point(L2[0],L2[1]),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(255,255,255),1);
	   cv::line(yellow_hue_image, cv::Point(L2[0],L2[1]), cv::Point(L2[2],L2[3]), cv::Scalar(255,0,0),2,CV_AA);
	   r_angle = angle;
	   }
	}
#endif 
  	   cmd->angular.z = angle;	
//         if (l_angle != 0 || r_angle != 0)ROS_INFO("Left=  %f,  Right= %f", l_angle,r_angle);

    // Draw an example circle on the video stream

      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", jout_image).toImageMsg();
      image_pub_.publish(out_img);
#ifdef TestVideo2
      sensor_msgs::Image::Ptr out_img2 = cv_bridge::CvImage(msg->header, "bgr8", yellow_hue_image).toImageMsg();
      image_pub2_.publish(out_img2);
#endif
      vel_pub_.publish(cmd);
  }
};

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "can_driving_node"); 

      can_dynamix::ImageConverter  videoCon ;
      ros::spin();

  return 0;
}
