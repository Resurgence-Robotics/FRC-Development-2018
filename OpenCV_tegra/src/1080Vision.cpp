#include <stdio.h>	// standard input and output stream
#include <stdlib.h>	// standard library for system level functions and low level application access
#include <string> 	//
#include <iostream>	// linux gcc application capabilities
#include <sstream>	// linux gcc runtime access
#include <unistd.h>	// universal standard numeric rules
//opencv includes
#include <opencv2/opencv.hpp>// all C++ libs for opencv algorithms
#include <opencv2/core.hpp>// data types specific to images and matrix functionality

using namespace cv;
using namespace std;

/*
 * 1080Vision.cpp
 *	Author: 1080 Resurgence Robotics
 *  Created on: Jul 30, 2017
 */
// functions to override
//Camera constants used for distance calculation
#define X_IMAGE_RES 320		//X Image resolution in pixels, should be 160, 320 or 640 Y image resolution is double for 2:1 aspect ratio
#define Y_IMAGE_RES 640
#define VIEW_ANGLEH 61		//MICROSOFT LIFECAM HD-5000
#define VIEW_ANGLEV 34.3	//MICROSOFT LIFECAM HD-5000
//#define VIEW_ANGLE 43.5  //Axis M1011 camera -- OUR IP CAM FROM KOP

//CLASSIFICATION OF THE OBJECT WE ARE PROCESSING used for target identification
//							*** units = pixel ***
#define ASPECT_RATIO_CONST 2.75 // FOR RECTANGULAR OBJECTS ONLY, TARGET WIDTH TO HIGHT RATIO
#define X_EDGE_LIMIT 3
#define Y_EDGE_LIMIT 4
#define CAM_ID 0
#define CAM_IP 10.10.80.250
string OriginalImage ("Samples/Original.png"); // will throw escape sequences in linux CHANGE FWDSLASH TO BKSLASH
string HSVfilteredImage ("Samples/HSVThresholdOutput.png");

//Minimum area of particles to be considered
#define AREA_MINIMUM 500// 500 pixels
// define the HSV threshold values for filtering our target
double Hue[] {60, 100};
double Sat[] {90, 255};
double Val[] {20, 255};	//HSV threshold criteria array, ranges are in that order H-low-high, S-low-high, V-low-high
//Edge profile constants of our shape
#define XMAXSIZE 300
#define XMINSIZE 24
#define YMAXSIZE 600
#define YMINSIZE 24
//Structure to represent the scores for the various tests used for target identification WILL BE USED LATER
	struct Scores
	{
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;

	};
void HSVThreshold(Mat &input, Mat &output,double Hue[],double Sat[], double Val[]);//created above main so it can be used, but defined later to keep from using up space

int main()
{
	//create videostream object and image
	// from stream
	VideoCapture Stream;
	Stream= VideoCapture(CAM_ID);//starts a camera stream
	//from image
	Mat Original(X_IMAGE_RES, Y_IMAGE_RES, CV_CN_MAX); ;
	Original= imread(OriginalImage);// loads an image from the file
	// create windows for viewing each step in filter process until we are able to run without them ie... "calibrated"

	while(true)//change later to something like "while an image exists"
	{

	//grab one image from file or stream
		Mat Frame(X_IMAGE_RES, Y_IMAGE_RES, CV_CN_MAX);// create a matrix of pixil data called frame THAT MATCHES CAMERA RESOLUTION
		//frame= Stream.grab();// from usb, grab a frame if new frame is available
		Frame= Original;// from filesystem
//process it-> WHEN EACH OF THE BELOW STEPS IS COMPLETE THEY SHOULD HAVE A CORRESPONDING WINDOW FOR PREVIEWS.
	//HSV filter
		Mat HSVThresholdOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_CN_MAX);// 2 channel
		HSVThreshold(Frame, HSVThresholdOutput,Hue,Sat,Val);// each step of the process should be like this
		imwrite(HSVfilteredImage,HSVThresholdOutput);
	//identify contours

	//convex hull the image

//perform algorithms described later to score and classify the contours.
		//sleep(.05);//20fps
		//break;
	}
	return 0;
}

void HSVThreshold(Mat &input, Mat &output,double Hue[],double Sat[], double Val[])// this function uses an input and output givin by the user allowing major flexibility
{
	cvtColor(input,output,cv::COLOR_BGR2HSV);// convert the image from color image channles to 2 channel HSV "black white", binary
	inRange(output, cv::Scalar(Hue[0],Sat[0],Val[0]),cv::Scalar(Hue[1],Sat[1],Val[1]),output);
}


