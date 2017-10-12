
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <std_msgs/UInt16.h>

#include "std_msgs/String.h"
#include "can_dynamix/lidarMsg.h"


#define QSIZE 10


using namespace std;
using namespace cv;


double init_x, x_m, x_p, x_hat, x_distance;
double init_y, y_m, y_p, y_hat, y_distance;
double final_x, final_y;
double init_angle, o_x, o_y, o_z, o_w, angle_p, angle_hat;
double object_angle;

float length, angle, final_angle;
int x, y;
int final_dot;

double middle_angle;

double object_x=2;
double object_y=2;


void lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanValue)
{


  float extract_dot[360];
  Mat Map(600,600,CV_8UC3,Scalar(0,0,0));

  for(int i=0; i<360;i++){
    length = (float)(100*(scanValue->ranges[i]));
    angle = (float)i * M_PI / 180.0;
//    if (length == 0) cout << "no data angle = " << i << " RAD = " << angle << endl;
//    if (length < 1) cout << "too close = " << i << " len = " << angle << endl;

    x=-(int)(length*sin(angle));  // x점
    y=-(int)(length*cos(angle));  // y점 
    line(Map, Point(300,300), Point(300+x,300+y), Scalar(255,255,0), 1);  // 라인표시 
    circle(Map, Point(300+x,300+y), 4, Scalar(155,155,155), -1,8,0); // 라인끝 표시 
    if (length < 150 && length > 100) circle (Map, Point(300+x,300+y), 4, Scalar(255,0,0),-1,8,0); // 가까운 점 표시 
    if (length <= 100) circle (Map, Point(300+x,300+y), 4, Scalar(0,0,255),-1,8,0); // 더 가까운 점 표시 
//    if (length <= 100 && length > 0) putText(Map,"Warnig ", cv::Point(300+x,300+y),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(0,0,255),1);
    line(Map, Point(300,300), Point(300,120), Scalar(0,0,255), 1);  // 헤드라인표시 
    line(Map, Point(300,300), Point(200,140), Scalar(0,0,255), 1);  // 좌측라인표시 
    line(Map, Point(300,300), Point(400,140), Scalar(0,0,255), 1);  // 우측라인표시 
    circle(Map, Point(300,300), 3, Scalar(0,0,255), -1,8,0); //center
//    circle( in_image, center, 3, color, -1, 8, 0 );   // center
    putText(Map,"head", cv::Point(290,150),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(255,255,255),1);
//    cout << "angle = " << i << " length = " << length << endl;
  }

  imshow("LIDAR MAP",Map);
  waitKey(1);

}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "can_lidar_disp_node");
  ros::NodeHandle n;
  ros::Publisher lidarDisp_pub = n.advertise<can_dynamix::lidarMsg>("can_dynamix/lidarDisp",10);
  ros::Subscriber lidarScan_sub = n.subscribe("scan", QSIZE, lidarScanCallback);

  ROS_INFO ("lidar_disp_node start");


  cv::Mat background;

  while (ros::ok())
  { 

    ros::spinOnce();
//    loop_rate.sleep();

  }


}

