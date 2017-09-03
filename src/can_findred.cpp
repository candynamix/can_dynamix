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


// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp
//

/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

/* can_dynamix team 
   JunHyung choi

   2017-09-03
*/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.h
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <dynamic_reconfigure/server.h>
#include <can_dynamix/FindredMsg.h>
//#define test
#define spin_once

using namespace cv;
using namespace std;


namespace can_dynamix {

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;



  cv::Mat src_gray,src_wh,red_hue_image,yellow_hue_image,green_hue_image,lower_red_hue_range,upper_red_hue_range;
  cv::Mat jout_image_r,jout_image_g,jout_image_y,test_image ;
  cv::Mat in_image;
  cv_bridge::CvImagePtr cv_ptr;
  bool debug_view_;
  int cols;



  int radius_g,radius_y,radius_r,radius_w;
  int x_point_g,x_point_y,x_point_r,x_point_w,x_point_last;
//  int RedLightDet, YellowLightDet, GreenLightDet;
//  int RedLightDet_chatt, YellowLightDet_chatt, GreenLightDet_chatt;
//  int vs_mode;

   int RedLightDet = 0;
   int YellowLightDet = 0;
   int GreenLightDet = 0;
   int RedLightDet_chatt = 0;
   int YellowLightDet_chatt = 0;
   int GreenLightDet_chatt = 0;
   int vs_mode = 0;


public:

  int redLight=0, yellowLight=0,greenLight=0;

  ImageConverter()
    : it_(nh_)
  {

     image_sub_ = it_.subscribe("/can_dynamix/video", 1,  &ImageConverter::imageCb, this);

 #ifdef test
     image_pub_ = it_.advertise("/can_dynamix/video_out", 1);

#endif
  //   ros::NodeHandle pnh_("~");
  //   pnh_.param("debug_view", debug_view_, false);
  }

  ~ImageConverter()
  {
    if( debug_view_) {
      cv::destroyWindow(OPENCV_WINDOW);
    }
  }
  void reportRGB();
  //void key_chattering();
 // void signMode();

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {

        /* HAU VALUE
          oRANGE 0-22
          YELLOW 22-38
          GREEN 38-75
          BLUE 75-130
          VIOET 130-160
          RED 160-179
         */

    try
    {

        in_image = cv_bridge::toCvShare(  msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::cvtColor( in_image, src_gray, cv::COLOR_BGR2HSV );

       // cv::inRange(src_gray, cv::Scalar(0, 0, 200), cv::Scalar(179,255, 255), src_wh);
      //  cv::GaussianBlur(src_wh, src_wh, cv::Size(9, 9), 2, 2);

       cv::inRange(src_gray, cv::Scalar(0, 100, 150), cv::Scalar(18, 255, 255), lower_red_hue_range);
       cv::inRange(src_gray, cv::Scalar(160, 100, 150), cv::Scalar(179, 255, 255), upper_red_hue_range);
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
       // cv::inRange(src_gray, cv::Scalar(170, 160, 150), cv::Scalar(179, 255, 255), red_hue_image);
        cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

        cv::inRange(src_gray, cv::Scalar(20,100, 150), cv::Scalar(30, 255, 255), yellow_hue_image);
        cv::GaussianBlur(yellow_hue_image, yellow_hue_image, cv::Size(9, 9), 2, 2);

        cv::inRange(src_gray, cv::Scalar(39, 100, 150), cv::Scalar(75, 255, 255),green_hue_image );
        cv::GaussianBlur(green_hue_image, green_hue_image, cv::Size(9, 9), 2, 2);


        cols =in_image.cols;  // image columms



      //  radius_w= ImageConverter::draw(src_wh,100,25,cv::Scalar(255,255,255));
        radius_r=ImageConverter::draw(red_hue_image,100,25,cv::Scalar(0,0,255));
        radius_y=ImageConverter::draw(yellow_hue_image,100,30,cv::Scalar(0,255,255));
        radius_g=ImageConverter::draw(green_hue_image,30,10,cv::Scalar(0,255,0));

        ImageConverter::key_chattering();
        ImageConverter::signMode();



    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

#ifdef test
     // sensor_msgs::Image::Ptr out_img_a = cv_bridge::CvImage(msg->header, "bgr8", jout_image).toImageMsg();
     // image_pub_a.publish(out_img_a);

        sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", in_image).toImageMsg();
        image_pub_.publish(out_img);

#endif

  }


int draw(cv::Mat source,int High_thres ,int low_thres,const cv::Scalar color)
{


       int x_point=0;
       int radius=0;
       int x1=0;
       int x2=0;
       std::vector<cv::Vec3f> circles; // 3 element vector of floats, this will be the pass by reference output of HoughCircles()

       cv::HoughCircles(source,           // input image
                        circles,                 // function output (must be a standard template library vector
                        CV_HOUGH_GRADIENT,       // two-pass algorithm for detecting circles, this is the only choice available
                        1,                        // size of image / this value = "accumulator resolution", i.e. accum res = size of image / 2
                        source.rows/2,    // min distance in pixels between the centers of the detected circles
                        High_thres,                     // high threshold of Canny edge detector (called by cvHoughCircles)
                        low_thres,                      // low threshold of Canny edge detector (set at 1/2 previous value)
                        2,                       // min circle radius (any circles with smaller radius will not be returned)
                       50);                   // max circle radius (any circles with larger radius will not be returned)






       for( size_t i = 0; i < circles.size(); i++ )
       {
           cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
           x_point = cvRound(circles[i][0]);


           x2=x_point_last+50;
           x1=x_point_last-50;
           x_point_last=x_point;

          if(x2 > cols)   x2=cols;
          else if(x1 < 0) x1=0;
          else

        // circle( in_image, center, 3, color, -1, 8, 0 );   // center

         if (cvRound(circles[i][2]) > 2 && (x1< x_point <x2) ){
         circle( in_image, center, radius, color, 5 );   // mark
         radius = cvRound(circles[i][2]);
          }
      }

      return radius ;


}




    void key_chattering()
    {

    //red
      if(radius_r > 2 )
           {
              if(RedLightDet == 1)
              {
                  if(RedLightDet_chatt > 2)
                  {
                          RedLightDet = 0;
                          //SW1 = 1;
                  }
                  else
                  {
                          RedLightDet_chatt++;
                  }
              }
          }
          else
          {
              if(RedLightDet_chatt == 0)
              {
                      RedLightDet = 1;
              }
              else
              {
                      RedLightDet_chatt--;

              }
          }

      //yellow
      if(radius_y > 2 )
              {
                 if(YellowLightDet == 1)
                 {
                     if(YellowLightDet_chatt > 2)
                     {
                             YellowLightDet = 0;
                           //SW2 = 1;
                     }
                     else
                     {
                             YellowLightDet_chatt++;
                     }
                 }
             }
             else
             {
                 if(YellowLightDet_chatt == 0)
                 {
                         YellowLightDet = 1;
                 }
                 else
                 {
                         YellowLightDet_chatt--;
                 }
             }


         //green
                if(radius_g > 2 )
                 {
                    if(GreenLightDet == 1)
                    {
                        if(GreenLightDet_chatt > 2)
                        {
                                GreenLightDet = 0;
                                //SW3 = 1;
                        }
                        else
                        {
                                GreenLightDet_chatt++;
                        }
                    }
                }
                else
                {
                    if(GreenLightDet_chatt == 0)
                    {
                            GreenLightDet = 1;
                    }
                    else
                    {
                            GreenLightDet_chatt--;

                    }
                }

    }


   void signMode()
  {
    /*
      ++vs_mode;
      if(vs_mode >= 2)
      {
              vs_mode = 0;
      }
     */


      switch(vs_mode)
      {

              case 0:if (GreenLightDet==1 && YellowLightDet==1 && RedLightDet==0) //red
                              {
                                  redLight=1;
                                  yellowLight=0;
                                  greenLight=0;
                                  vs_mode=1;


                              }
                    else if (GreenLightDet==1 && YellowLightDet==0 && RedLightDet==1) //yellow
                              {
                                  redLight=0;
                                  yellowLight=1;
                                  greenLight=0;
                                  vs_mode=0;



                              }
                    else if(GreenLightDet==0 && YellowLightDet==1 && RedLightDet==1) //green
                              {
                                  redLight=0;
                                  yellowLight=0;
                                  greenLight=1;
                                  vs_mode=0;


                              }
                    else if(GreenLightDet==1 && YellowLightDet==1 && RedLightDet==1)  //no sign
                              {
                                  redLight=0;
                                  yellowLight=0;
                                  greenLight=1;
                                   vs_mode=0;


                              }

                                  break;
              case 1:if(GreenLightDet==0 && YellowLightDet==1 && RedLightDet==1) //green
                              {
                                  redLight=0;
                                  yellowLight=0;
                                  greenLight=1;
                                  vs_mode=0;

                              }




      }

 }

};

    void ImageConverter::reportRGB()
    {

        ROS_INFO("send r %d,y %d,g %d,redLight %d,yellowLight %d,greenLight %d",radius_r,radius_y,radius_g,redLight,yellowLight,greenLight);
        ROS_INFO("send RedLightDet_chatt %d,vs_mode %d,GreenLightDet_chatt %d",RedLightDet_chatt,vs_mode,GreenLightDet_chatt);
       // ROS_INFO("msg %d",sign_msg);

    }



}



int main(int argc, char **argv) 
{
  ros::init(argc, argv, "can_findred");
  ros::NodeHandle nh_;
  ros::Publisher pub_sign = nh_.advertise<can_dynamix::FindredMsg>("/can_dynamix/findred", 100);
  can_dynamix::ImageConverter imageexec;
  can_dynamix::FindredMsg sign_msg;

#ifdef spin_once
 ros::Rate loop_rate(10);
#endif
  while (ros::ok())
  { 

     if (imageexec.redLight==1 ||imageexec.yellowLight==1)  sign_msg.sign_result="stop";
     else if (imageexec.greenLight==1) sign_msg.sign_result="go";

     pub_sign.publish(sign_msg);

#ifdef test
     imageexec.reportRGB();
#endif

#ifdef  spin_once
    ros::spinOnce();
    loop_rate.sleep();

#else
       ros::spin();
#endif

  }

  return 0;
}
