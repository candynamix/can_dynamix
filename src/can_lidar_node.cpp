
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <std_msgs/UInt16.h>

#include "std_msgs/String.h"

#define test
#define QSIZE 10
#define disp_lidar 1
#define WAITKEYSIZE 
//#define LIDAR_MODE


#define NORMAL_MODE 0
#define PARKING_MODE 1
#define LIDAR_MODE 2
#define NORMAL_TO_PARKING 3

int robotMode = NORMAL_MODE;

using namespace std;
using namespace cv;



/*
double object_angle;
float length, angle, final_angle;
int x, y;
*/

int start_ok=1;
double init_x, x_m, x_p, x_hat, x_distance;
double init_y, y_m, y_p, y_hat, y_distance;
double final_x, final_y;
double init_angle, o_x, o_y, o_z, o_w, angle_p, angle_hat;

double object_angle;

float length, angle, final_angle;
int x, y;

int angle_D;


void lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanValue)
{

   #ifdef test	
   cout << "angle_min  " << scanValue -> angle_min<< "         deg =" <<  scanValue -> angle_min *180/3.14 << endl;
   cout << "angle_max  " << scanValue -> angle_max<< "   deg =" <<  scanValue -> angle_max *180/3.14 << endl;
   cout << "angle_increment  " << scanValue -> angle_increment<< endl;
//   cout << "scan_time  " << scanValue -> scan_time<< endl;
   cout << "range_min  " << scanValue -> range_min<< endl;
   cout << "range_max  " << scanValue -> range_max<< endl;
   cout << "range size  " << sizeof(scanValue -> ranges) << endl;

   cout << "range dist 359= " << scanValue -> ranges[359] << endl;
   cout << "range dist   1= " << scanValue -> ranges[1] << endl;
   cout << "range dist   2= " << scanValue -> ranges[2] << endl;
   cout << "range dist  31= " << scanValue -> ranges[31] << endl;
   cout << "range dist  32= " << scanValue -> ranges[32] << endl;
   cout << "range dist  61= " << scanValue -> ranges[61] << endl;
   cout << "range dist  62= " << scanValue -> ranges[62] << endl;
   cout << "range dist  91= " << scanValue -> ranges[91] << endl;
   cout << "range dist  92= " << scanValue -> ranges[92] << endl;
   cout << "range dist  93= " << scanValue -> ranges[93] << endl;
   cout << "range dist 179= " << scanValue -> ranges[179] << endl;
   cout << "range dist 181= " << scanValue -> ranges[181] << endl;
   cout << "range dist 182= " << scanValue -> ranges[182] << endl;

   cout << "range dist 271= " << scanValue -> ranges[271] << endl;
   cout << "range dist 272= " << scanValue -> ranges[272] << endl;
   cout << "range dist 273= " << scanValue -> ranges[273] << endl;

   cout << "range dist 301= " << scanValue -> ranges[301] << endl;
   cout << "range dist 302= " << scanValue -> ranges[302] << endl;
   cout << "range dist 303= " << scanValue -> ranges[303] << endl;


   cout << "range dist 329= " << scanValue -> ranges[329] << endl;
   cout << "range dist 331= " << scanValue -> ranges[331] << endl;
   cout << "range dist 332= " << scanValue -> ranges[332] << endl;
   #endif

  double lidar_angle;


  for ( angle_D = 330; angle_D < 360; angle_D++) {   // 전방 우측면 30도각 탐지
    length = (float)((scanValue->ranges[angle_D]));
    #ifdef test
    	ROS_INFO("F-Right [%d] length [%f] ", angle_D, length); 
    #endif 
  }

  for (angle_D = 0; angle_D < 30; angle_D++) { // 전방 좌측면 30도각 탐지 
    length = (float)((scanValue->ranges[angle_D]));
    #ifdef test
    	ROS_INFO("F-Left  [%d] length [%f] ", angle_D, length);  
    #endif 
  }


  for ( angle_D = 240; angle_D < 300; angle_D++) {   // 우측면 60도각 탐지
    length = (float)((scanValue->ranges[angle_D]));
    #ifdef test
    	ROS_INFO("Right [%d] length [%f] ", angle_D, length); 
    #endif 
  }

}


int main (int argc, char **argv)
{

  ros::init(argc, argv, "can_lidar_node");
  ros::NodeHandle n;
  ros::Publisher lidar_pub = n.advertise<std_msgs::String>("can_dynamix/lidar",10);
  ros::Subscriber lidarScan_sub = n.subscribe("scan", QSIZE, lidarScanCallback);
  ros::spin();
  return 0;
}
