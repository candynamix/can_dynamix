
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*objectTrackingTutorial.cpp

Written by  Kyle Hounslow 2013

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*/
//https://www.youtube.com/watch?v=4KYlHgQQAts
//code : https://www.dropbox.com/s/o22cnih7v0mu7gv/multipleObjectTracking.cpp?dl=0

/*
 * can_dynamix Team
 * HakSeung Wang
 * 2017-09-03
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "can_dynamix/VelMsg.h"
#include "can_dynamix/Object.h"

//#define test

using namespace cv;
using namespace std;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;//40*40
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;//1.5
/////////////////////////////////////////////////////////////////////
int result_index = 0;
int last_y = 0;
int rows = 0;

void drawObject(vector<Object> Objects, Mat &frame){

	for(int i = 0; i < Objects.size(); i++){

	cv::circle(frame,cv::Point(Objects.at(i).getXPos(), Objects.at(i).getYPos()),10,cv::Scalar(0,0,255));
	cv::putText(frame,
                    format("(X:%d, Y:%d, num:%d)",Objects.at(i).getXPos(), Objects.at(i).getYPos(), i),
                    cv::Point(Objects.at(i).getXPos(), Objects.at(i).getYPos()+20),
                    1,
                    1,
                    Scalar(0,255,0));
	cv::putText(frame,
                    Objects.at(i).getType(),
                    cv::Point(Objects.at(i).getXPos(), Objects.at(i).getYPos()-30),
                    3,
                    1,
                    Objects.at(i).getColour());
        //ROS_INFO("%d번째, XPos : %d, YPos : %d", i, Objects.at(i).getXPos(), Objects.at(i).getYPos());
}
}

void detectObject(vector<Object> Objects){
    int y1 = 0, y2 = 0;

    for(int i = 0; i < Objects.size(); i++){

        y2 = last_y + 20;
        y1 = last_y - 20;

        if(y2 > rows) y2 = rows;
        else if(y1 < 0) y1 = 0;
        //if(i !=1)result_index = 0;
        if(i == 1)
        {
            last_y = Objects.at(1).getYPos();
        if(Objects.at(0).getYPos() >= y1 && Objects.at(0).getYPos() <= y2){
            result_index = abs(Objects.at(0).getXPos()-Objects.at(1).getXPos());
        }}

  /* if(i == 1)
    {
            result_index = abs(Objects.at(0).getXPos()-Objects.at(1).getXPos());
            ROS_INFO("result : %d", result_index);
    }*/
    }

}
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);



}
void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed){
	
	vector<Object> Objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
                                ROS_INFO("index : %d", index);
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object blockbar;				
					
					blockbar.setXPos(moment.m10/area);
					blockbar.setYPos(moment.m01/area);
					Objects.push_back(blockbar);


					objectFound = true;

				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawObject(Objects,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}
void trackFilteredObject(Object attribute, Mat threshold,Mat HSV, Mat &cameraFeed){
	
	vector<Object> Objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
              //  ROS_INFO("Objects : %d", numObjects);
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
 //             ROS_INFO("index : %d", index);
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object blockbar;				
					
					blockbar.setXPos(moment.m10/area);
					blockbar.setYPos(moment.m01/area);
					blockbar.setType(attribute.getType());
					blockbar.setColour(attribute.getColour());
					
					Objects.push_back(blockbar);
                          //              ROS_INFO("XPos : %d, YPos : %d", blockbar.getXPos(), blockbar.getYPos());
					objectFound = true;

				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawObject(Objects,cameraFeed);}
                                result_index = 0;
                                detectObject(Objects);
		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}

}

namespace can_dynamix {

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
#ifdef test
  image_transport::Publisher image_pub_;
#endif
  bool debug_view_;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("cv_camera/image_raw", 100,  &ImageConverter::imageCb, this); //can_dynamix/video,cv_camera/image_raw
#ifdef test
    image_pub_ = it_.advertise("/can_dynamix/video_blockbar", 1);
#endif

    ros::NodeHandle pnh_("~");
    pnh_.param("debug_view", debug_view_, false);
  }

  ~ImageConverter()
  {
    if( debug_view_) {
      cv::destroyWindow(OPENCV_WINDOW);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {    
    bool calibrationMode = false;//true, false
    Mat in_image;
    Mat threshold;
    Mat HSV_image;

    try
    {
       in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        rows = in_image.rows;
        cvtColor(in_image, HSV_image, COLOR_BGR2HSV);

	if(calibrationMode==true){
            imshow("Source", in_image);
            waitKey(3);
            //create window for trackbars
            //create trackbars and insert them into window
            //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
            //the max value the trackbar can move (eg. H_HIGH),
            //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
            //
            createTrackbar( " H_MIN :", "Source", &H_MIN, H_MAX, 0);
            createTrackbar( " H_MAX :", "Source", &H_MAX, H_MAX, 0);
            createTrackbar( " S_MIN :", "Source", &S_MIN, S_MAX, 0);
            createTrackbar( " S_MAX :", "Source", &S_MAX, S_MAX, 0);
            createTrackbar( " V_MIN :", "Source", &V_MIN, V_MAX, 0);
            createTrackbar( " V_MAX :", "Source", &V_MAX, V_MAX, 0);
	//if in calibration mode, we track objects based on the HSV slider values.
	cvtColor(in_image,HSV_image,COLOR_BGR2HSV);
	inRange(HSV_image,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
	morphOps(threshold);
	imshow("calibration", threshold);
        waitKey(3);
	trackFilteredObject(threshold,HSV_image,in_image);
	}else{

        Object blockbar("Block Bar");//, traffic_light("traffic_light");

	cvtColor(in_image,HSV_image,COLOR_BGR2HSV);
	inRange(HSV_image,blockbar.getHSVmin(),blockbar.getHSVmax(),threshold);
	morphOps(threshold);
        trackFilteredObject(blockbar,threshold,HSV_image,in_image);
}
#ifdef test
        imshow("Result", in_image);
        waitKey(3);
#endif
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    
//       ROS_INFO("processing");
    ROS_INFO("result : %d", result_index);

#ifdef test
      sensor_msgs::Image::Ptr out_img1 = cv_bridge::CvImage(msg->header, "bgr8", in_image).toImageMsg();
      image_pub_.publish(out_img1);
#endif
  }
};

}

int main(int argc, char **argv) 
{

  ros::init(argc, argv, "can_dynamix_blockbar_MCU");

  ros::NodeHandle nh_;
  ros::Publisher pub_sign = nh_.advertise<can_dynamix::VelMsg>("/can_dynamix/blockbar", 100);
  can_dynamix::ImageConverter imageCon ;
  can_dynamix::VelMsg sign_msg;

 ros::Rate loop_rate(5);

  while (ros::ok())
  {
  // can_dynamix::ImageConverter imageCon ;
    if(result_index >= 90 && result_index <=130)
      { sign_msg.sign_result="stop"; }
    else
    {
        sign_msg.sign_result="go";
    }
        pub_sign.publish(sign_msg);
#ifdef test
    ROS_INFO("processing");
#endif
    ros::spinOnce();
   // ros::spin();
    loop_rate.sleep();


  }

  return 0;
}
