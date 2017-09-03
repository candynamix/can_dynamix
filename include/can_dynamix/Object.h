#pragma once

#include <string>
#include <vector>

//#include "ros/ros.h"

//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgcodecs/imgcodecs.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

class Object
{
public:
	Object(void);
	~Object(void);

	Object(string name);
	
	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	Scalar getHSVmin();
	Scalar getHSVmax();

	void setHSVmin(Scalar min);
	void setHSVmax(Scalar max);

	string getType(){return type;}
	void setType(string t){type = t;}

	Scalar getColour(){
		return Colour;
	}

	void setColour(Scalar c){
		Colour = c;
	}

private:

	int xPos, yPos;
	string type;
	Scalar HSVmin, HSVmax;
	Scalar Colour;


};


