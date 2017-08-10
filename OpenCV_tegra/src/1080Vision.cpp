#include <stdio.h>	// standard input and output stream
#include <stdlib.h>	// standard library for system level functions and low level application access
#include <string> 	// library for using the string data type
#include <iostream>	// linux gcc application io
#include <sstream>	// linux gcc runtime access
#include <unistd.h>	// universal standard numeric rules
//opencv includes
#include <opencv2/opencv.hpp>// all C++ libs for opencv algorithms
#include <opencv2/core.hpp>// data types specific to images and matrix functionality, as well as matrix arithmetic

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
//image locations
#define CAM_ID 0 // usb camera
#define CAM_IP 10.10.80.250 // ip camera
string OriginalImage ("Samples/Original.png"); // test sample images
string HSVfilteredImage ("Samples/HSVThresholdOutput.png");
string ContoursImage ("Samples/ContoursImage.png");
string filteredImage("Samples/ContoursImage.png");
//CLASSIFICATION OF THE OBJECT WE ARE PROCESSING used for target identification
//							*** units = pixel ***
#define ASPECT_RATIO_CONST 2.75 // FOR RECTANGULAR OBJECTS ONLY, TARGET WIDTH TO HIGHT RATIO
//Edge profile constants of our shape
#define XMAXSIZE 300
#define XMINSIZE 24
#define YMAXSIZE 600
#define YMINSIZE 24

//Minimum area of particles to be considered
#define AREA_MINIMUM 500// 500 pixels
// define the HSV threshold values for filtering our target //HSV threshold criteria array, ranges are in that order H-low-high, S-low-high, V-low-high
double Hue[] {75, 100};
double Sat[] {90, 255};
double Val[] {60, 255};

//Structure to represent the scores for the various tests used for target identification WILL BE USED LATER
	struct Scores
	{
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;

	};
	typedef vector<vector<Point> > TContours;
	typedef vector<Point> UContour;
void HSVThreshold(Mat &input, Mat &output,double Hue[],double Sat[], double Val[]);//created above main so it can be used, but defined later to keep from using up space
void GetContours(Mat &input, TContours &contours);
void FilterContours(TContours &input, TContours &output);
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
		Mat frame(X_IMAGE_RES, Y_IMAGE_RES, CV_CN_MAX);// create a matrix of pixil data called frame THAT MATCHES CAMERA RESOLUTION
		//frame= Stream.grab();// from usb, grab a frame if new frame is available
		frame= Original;// from filesystem
	// WHEN EACH OF THE BELOW STEPS IS COMPLETE, create a window or new image for validation.
	//HSV filter
		Mat HSVThresholdOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC1);// 1 channel binary
		HSVThreshold(frame, HSVThresholdOutput,Hue,Sat,Val);// each step of the process should be like this
	//identify contours
		TContours contours; //created empty vector
		GetContours(HSVThresholdOutput, contours);
		PrintVector(contours);
	//Filter contours
//
//		TContours finalContours;
//		FilterContours(contours, finalContours);



	//	waitKey();// waits to exit program until we have pressed a key
		break;
	}
	return 0;
}
void PrintVector(TContours &input)
{
	for (int i= 0; i< input.end(); i++)
	{
		printf("%i, ", input[i]);
	}
}
void HSVThreshold(Mat &input, Mat &output, double Hue[], double Sat[], double Val[])// this function uses an input and output givin by the user allowing major flexibility
{
	cvtColor(input,output,cv::COLOR_BGR2HSV);// convert the image from color image channles to 2 channel HSV "black white", binary
	inRange(output, cv::Scalar(Hue[0],Sat[0],Val[0]),cv::Scalar(Hue[1],Sat[1],Val[1]),output);
	imwrite(HSVfilteredImage,output);// create a new image to show the processing worked
}

void GetContours(Mat &input, TContours &contours)
{
	Mat CannyOutput;
	Canny(input, CannyOutput, X_IMAGE_RES, Y_IMAGE_RES);
	vector<Vec4i> hierarchy;
	findContours(CannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	imwrite(ContoursImage, CannyOutput);
}

void FilterContours(TContours &input, TContours &output)
{
	double minArea = 300.0;  // default Double
	double minPerimeter = 0.0;  // default Double
	double minWidth = 0.0;  // default Double
	double maxWidth = 1000.0;  // default Double
	double minHeight = 0.0;  // default Double
	double maxHeight = 1000.0;  // default Double
	double solidity[] = {20, 100};
	double maxVertices = 1000000.0;  // default Double
	double minVertices = 0.0;  // default Double
	double minRatio = 0.0;  // default Double
	double maxRatio = 1000.0;  // default Double
	TContours hull;
	UContour shape;
	output.clear();
	Mat OutputImage(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC1);
	vector<Vec4i> hierarchy;

	for (UContour contour: input)
			{
				Rect bb = boundingRect(input);
				if (bb.width < minWidth || bb.width > maxWidth) continue;
				if (bb.height < minHeight || bb.height > maxHeight) continue;
				double area = contourArea(input);
				if (area < minArea) continue;
				if (arcLength(input, true) < minPerimeter) continue;
				convexHull(Mat(input, true), hull);
				double solid = 100 * area / contourArea(hull);
				if (solid < solidity[0] || solid > solidity[1]) continue;
				if (input.size() < minVertices || input.size() > maxVertices)	continue;
				double ratio = (double) bb.width / (double) bb.height;
				if (ratio < minRatio || ratio > maxRatio) continue;
				output.push_back(contour);

			}
			drawContours(OutputImage, hull, -1, 255, CV_FILLED);
			imshow(filteredImage, OutputImage);
}
