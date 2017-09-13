#include <sys/types.h>
#ifdef __WIN32__
# include <winsock2.h>// windows
#else
# include <sys/socket.h>// linux
#endif
//system includes
#include <stdio.h>	//
#include <stdlib.h>	//
#include <string> 	//
#include <iostream>	//
#include <sstream>	//
#include <unistd.h>	//
#include <math.h>
//opencv includes
#include <opencv2/opencv.hpp>// all C++ libs for opencv algorithms

using namespace cv;
using namespace std;

/*
 * 1080Vision.cpp
 *	Author: 1080 Resurgence Robotics
 *  Created on: Jul 30, 2017
 */

// functions to override
//Camera constants used for distance calculation
#define X_IMAGE_RES 640		//X Image resolution in pixels, should be 160, 320 or 640 Y image resolution is double for 2:1 aspect ratio
#define Y_IMAGE_RES 320
#define VIEW_ANGLE_X 61		//MICROSOFT LIFECAM HD-5000
#define VIEW_ANGLE_Y 34.3	//MICROSOFT LIFECAM HD-5000
//#define VIEW_ANGLE 43.5  //Axis M1011 camera -- OUR IP CAM FROM KOP
//image locations
#define CAM_ID 0 // usb camera
#define CAM_IP 10.10.80.250 // ip camera
string WebServer("roboRIO-1080-FRC.local");
string OriginalImage ("Samples/Original.png"); // test sample images
string RGBModImage("Samples/RGBModImage.png");
string HSVfilteredImage ("Samples/HSVThresholdOutput.png");
string ContoursImage ("Samples/ContoursImage.png");
string filteredImage("Samples/FilteredImage.png");
ofstream LOGFILE("LOG.csv");
//CLASSIFICATION OF THE OBJECT WE ARE PROCESSING used for target identification
//							*** units = pixel ***
#define ASPECT_RATIO_CONST 2.75 // FOR RECTANGULAR OBJECTS ONLY, TARGET WIDTH TO HIGHT RATIO
//Edge profile constants of our shape
#define Twidth 2 //in
#define Theight 5 //in
#define Tarea Twidth*Theight
#define XMAXSIZE 300
#define XMINSIZE 24
#define YMAXSIZE 600
// define the HSV threshold values for filtering our target //HSV threshold criteria array, ranges are in that order H-low-high, S-low-high, V-low-high
double Hue[] {0, 94};
double Sat[] {0, 114};
double Val[] {216, 255};
double minArea = 300.0;
double minPerimeter = 0.0;
double minWidth = 0.0;
double maxWidth = 1000.0;
double minHeight = 0.0;
double maxHeight = 1000.0;
double solidity[] = {20, 100};
double maxVertices = 1000000.0;
double minVertices = 0.0;
double minRatio = 0.0;
double maxRatio = 1000.0;

typedef vector<vector<Point> > ShapeArray; //  an array of contours that make up a shape[1],[2] = [(x,y), (x,y)...], [(x,y), (x,y)...]
typedef vector<Point> Points;// an Array of continuous points that form a single contour [1],[2]= [x,y],[x,y]

float Map(float x, float in_min, float in_max, float out_min, float out_max)
{
	// use this function to match two value's scales proportionally
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
float Limit(float num)
{
	   if (num > 1.0)
	   {
			   return 1.0;
	   }
	   if (num < -1.0)
	   {
			   return -1.0;
	   }
	   return num;
}
//Structure to represent the scores for the various tests used for target identification WILL BE USED LATER
struct Report
{
	int GoalCount;
	Point Center;
	double xEdge;
	double yEdge;
	double Distance;
	double Skew;
	float Angle;
	double PointOfAim;
};

void HSVThreshold(Mat &input, Mat &output,double Hue[],double Sat[], double Val[]);//created above main so it can be used, but defined later to keep from using up space
void GetContours(Mat &input, ShapeArray &contours);
void FilterContours(ShapeArray &input, ShapeArray &output, Mat &OutputImage);
void GenerateTargetReport(ShapeArray &input, Report Goal[], Mat &FilteredGoals);
void Write_csv( Report Goal[], ofstream &output);
void ModifyRGB(float intensity, bool reduceR, bool reduceG,  bool reduceB, Mat &input);
//void GlobalFieldPosition(ShapeArray &inputImage, float &GyroHeading, float &FieldX, float &FieldY);
void SendRobotDataUDP();
int main()
{
	//create videostream object and image
	// from stream
	VideoCapture Stream;
	Stream= VideoCapture(CAM_ID);//starts a camera stream
	Stream.set(CV_CAP_PROP_FRAME_WIDTH, X_IMAGE_RES);
	Stream.set(CV_CAP_PROP_FRAME_HEIGHT, Y_IMAGE_RES);
	//from image
	Mat Original(X_IMAGE_RES, Y_IMAGE_RES, CV_CN_MAX);
	Original= imread(OriginalImage);// loads an image from the file

	while(true)//change later to something like "while an image exists"
	{

	//Grab Image
		Mat frame(Y_IMAGE_RES, X_IMAGE_RES, CV_8UC3);// create a matrix of pixil data called frame THAT MATCHES CAMERA RESOLUTION
		Stream.retrieve(frame);// from usb, grab a frame if new frame is available
		//frame= Original;// from filesystem
		//modify RGB
		imshow("one",frame);
		//ModifyRGB(0.0, true, false, true, frame);
		//HSV filter
		Mat HSVThresholdOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC3);// 1 channel binary
		HSVThreshold(frame, HSVThresholdOutput,Hue,Sat,Val);// each step of the process should be like this
	//identify contours
		ShapeArray contours; //created empty vector
		GetContours(HSVThresholdOutput, contours);
	//Filter contours
		ShapeArray Goals;
		Mat FilteredOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC1);
		FilterContours(contours, Goals, frame);// returning image of different size?
	//calculate and score object
		Report GoalReport[Goals.size()];
		GenerateTargetReport(Goals, GoalReport,frame);
		imshow("final image", frame);
	//report via UDP and CSV
		//csv
		Write_csv(GoalReport,LOGFILE);
		usleep(0.1);//10 times a second
		waitKey(1);// waits to exit program until we have pressed a key
	}
	return 0;
}

void HSVThreshold(Mat &input, Mat &output, double Hue[], double Sat[], double Val[])// this function uses an input and output givin by the user allowing major flexibility
{
	cvtColor(input,output,COLOR_BGR2HSV);// convert the image from color image channles to 2 channel HSV "black white", binary
	inRange(output, Scalar(Hue[0],Sat[0],Val[0]),Scalar(Hue[1],Sat[1],Val[1]),output);
	imwrite(HSVfilteredImage,output);// create a new image to show the processing worked
}

void GetContours(Mat &input, ShapeArray &contours)
{
	Mat CannyOutput;
	Canny(input, CannyOutput, X_IMAGE_RES, Y_IMAGE_RES);
	vector<Vec4i> hierarchy;
	findContours(CannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	imwrite(ContoursImage, CannyOutput);
}

void FilterContours(ShapeArray &input, ShapeArray &output, Mat &OutputImage )
{

	ShapeArray hull( input.size() );


	output.clear();
	int sizeOfArray=int(input.size())-1;
	vector<Vec4i> hierarchy;

	for (int i=0; i<= sizeOfArray; i++)
			{
				Points store =input[i];
				Rect bb = boundingRect(store);
				if (bb.width < minWidth || bb.width > maxWidth) continue;
				if (bb.height < minHeight || bb.height > maxHeight) continue;
				double area = contourArea(store);
				if (area < minArea) continue;
				if (arcLength(store, true) < minPerimeter) continue;
				convexHull( Mat(store,true), hull[i], false );
				double solid = 100 * area / contourArea(hull[i]);
				if (solid < solidity[0] || solid > solidity[1]) continue;
				if (store.size() < minVertices || store.size() > maxVertices)	continue;
				double ratio = (double) bb.width / (double) bb.height;
				if (ratio < minRatio || ratio > maxRatio) continue;
				output.push_back(store);
				store.clear();

			}
			drawContours(OutputImage, hull, -1, 255, CV_FILLED);
			imwrite(filteredImage, OutputImage);
}

void GenerateTargetReport(ShapeArray &input, Report Goal[], Mat &FilteredGoals)
{
	int sizeOfArray=int(input.size());// cast to signed int and subtract 1 to prevent using a number outside the scope of the array
	if(sizeOfArray==0)
	{
		printf("0 goal(s), \n");
		return;
	}
	for(int i=0; i<sizeOfArray; i++)
	{
		Rect bb = boundingRect(input[i]);
		Goal[i].Center.x = bb.x + (bb.width/2);
		Goal[i].Center.y = bb.y + (bb.height/2);
		Goal[i].xEdge = bb.width;
		Goal[i].yEdge = bb.height;
		Goal[i].GoalCount = sizeOfArray;
		Goal[i].PointOfAim = Map(Goal[i].Center.x,0,X_IMAGE_RES,-1,1);
		Goal[i].Distance = (Theight / (tan(bb.height * (VIEW_ANGLE_Y / Y_IMAGE_RES) * (3.14159 / 180))));
		Goal[i].Angle = acos(2.0 * (Goal[i].Distance * (tan(float(bb.width) * (float(VIEW_ANGLE_X) / float(X_IMAGE_RES)) * (3.14159 / float(180)))/2)) / float(Twidth / 2));
		printf("%i goal(s), Goal[%i] Center is: [%i,%i], POI is:[%f], Distance is:[%f], Angle is:[%f] \n", Goal[i].GoalCount, i+1, Goal[i].Center.x, Goal[i].Center.y, Goal[i].PointOfAim, Goal[i].Distance, Goal[i].Angle);
	}

}

void SendRobotDataUDP()
{

}

void Write_csv(Report Goal[], ofstream &output)
{
	//int i=1;
	//Point cell;

	//int sizeOfArray=int(Goal);// cast to signed int and subtract 1 to prevent using a number outside the scope of the array

}

void ModifyRGB(float intensity, bool reduceR, bool reduceG,  bool reduceB, Mat &input)
{
	printf("cols: %i, rows: %i \n", input.cols,input.rows);
	for(int row = 0; row <=Y_IMAGE_RES; row++)
	{
		for(int col = 0; col <= X_IMAGE_RES; col++)
		{
			for(int color=0;color<3;color++ )//DID IT TO ALL 3 COLORS JUST FOR THE LOLS
			{

					int value = input.at<Vec3i>(row, col)[color];
					value=rint(value*intensity);
					printf("%i,%i,%i \n",row, col, value );
			}
//			if(reduceG)
//			{
//				input.at<Vec3i>(row, col)[1] /= intensity;
//			}
//			if(reduceR)
//			{
//				input.at<Vec3i>(row, col)[2] /= intensity;
//			}

		}
	}
	imwrite(RGBModImage, input);
}
