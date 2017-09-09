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
// https://github.com/robotpilot
/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

/*
   can_dynamix team 
   JinYoung Kim

   2017-09-03
*/


#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
//#include <geometry_msgs/Twist.h>
//#include "std_msgs/MultiArrayLayout.h"
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/MultiArrayDimension.h>

//#include <QTransform>

#define QSIZE 500


//using namespace std; 
//namespace can_dynamix{

//#define test

	std::string cmsg, lmsg, rmsg, bmsg;
	float mmsg;
        int mark_id;

void mark_Callback(const std_msgs::Float32MultiArray & msg)
{

    if(msg.data.size())
    {
		for(unsigned int i=0; i<msg.data.size(); i+=12)
		{
			// get data
			mark_id = (int)msg.data[i];
			float objectWidth = msg.data[i+1];
			float objectHeight = msg.data[i+2];

	ROS_INFO("mark detected message [%d] ", mark_id); 
//	ROS_INFO("markdata 0  [%d] ", (int)msg.data[0]); 
//	ROS_INFO("markdata 0  [%f] ", msg.data[0]); 
//	ROS_INFO("markdata 1  [%f] ", objectWidth); 
//	ROS_INFO("markdata 2  [%f] ", objectHeight); 
//	ROS_INFO("markdata 3  [%f] ", msg.data[3]); 
//	ROS_INFO("markdata 4  [%f] ", msg.data[4]); 
//	ROS_INFO("markdata 5  [%f] ", msg.data[5]); 
//	ROS_INFO("markdata 6  [%f] ", msg.data[6]); 
//	ROS_INFO("markdata 7  [%f] ", msg.data[7]); 
//	ROS_INFO("markdata 8  [%f] ", msg.data[8]); 
	ROS_INFO("mark data  9  [%f] ", msg.data[9]); 
	ROS_INFO("mark data 10  [%f] ", msg.data[10]); 
		}
    }
    else
    {
        mark_id = 0;
//    	printf("No objects detected.\n");
    }

}


int main(int argc, char **argv)
{

	ros::init(argc,argv,"can_findmark");
	ros::NodeHandle n;
	ros::Publisher msg_pub = n.advertise<std_msgs::String>("can_dynamix/findmark",QSIZE);
	ros::Subscriber mark_sub = n.subscribe("objects", 1, mark_Callback);
	ros::Rate loop_rate(100);

//        mmsg = 0.0;
	while(ros::ok())
	{
	   std_msgs::String msg;
	   std::stringstream ss;
	   ss<< "This Topic message from  Can_dynamix control ";
	   msg.data = ss.str();  
//	   if(mark_id == 1) ROS_INFO (" Parking AREA !! ") ;
//	   if(mark_id == 3) ROS_INFO (" TUNELL AREA !! ") ;
	   msg_pub.publish(msg);

	   ros::spinOnce();
	   loop_rate.sleep();
//	   ROS_INFO(" Sleep ");
	}
}
