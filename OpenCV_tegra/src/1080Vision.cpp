
/*
 *  1080Vision.cpp
 *	Author: 1080 Resurgence Robotics
 *  Created on: Jul 30, 2017
 */

#ifdef WIN32
	#include <winsock.h>
	#include <winsock2.h>
	typedef int socklen_t;
#else
	#include <unistd.h>
	#include <netdb.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <sys/wait.h>
#endif

#include <sys/types.h>
#include <stdio.h>	//
#include <stdlib.h>	//
#include <string> 	//
#include <iostream>	//
#include <sstream>	//
#include <unistd.h>	//
#include <math.h>	//
#include <stdio.h>
//opencv includes
#include <opencv2/opencv.hpp>// all C++ libs for opencv algorithms
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// network tables


using namespace cv;
using namespace std;
#define PORT 1086
#define SO_REUSEPORT 15

// functions to override

//Camera constants used for distance calculation
#define X_IMAGE_RES 640		//X Image resolution in pixels, should be 160, 320 or 640 Y image resolution is double for 2:1 aspect ratio
#define Y_IMAGE_RES 360
#define VIEW_ANGLE_X 61		//MICROSOFT LIFECAM HD-5000
#define VIEW_ANGLE_Y 34.3	//MICROSOFT LIFECAM HD-5000
//image locations
//#define CAM_ID 0 // usb camera
#define CAM_IP 10.10.80.25 // ip camera
int CAM_ID = 0;
bool camSuccess = false;
string RoborioDHCP("roboRIO-1080-FRC.local");
string TegraDHCP("tegra-ubuntu.local");
string CameraIP("http://10.10.80.12/mjpg/video.mjpg");
//http://10.10.80.12/mjpg/video.mjpg
string OriginalImage ("Samples/Original.png"); // test sample images
string RGBModImage("Samples/RGBModImage.png");
string HSVfilteredImage ("Samples/HSVThresholdOutput.png");
string ContoursImage ("Samples/ContoursImage.png");
string filteredImage("Samples/FilteredImage.png");
ofstream LOGFILE("LOG.csv");
//CLASSIFICATION OF THE OBJECT WE ARE PROCESSING used for target identification

//							*** units = pixel ***

//Edge profile constants of our shape
#define Twidth 2 //in
#define Theight 5 //in
#define TRatio Twidth/Theight
// define the HSV threshold values for filtering our target //HSV threshold criteria array, ranges are in that order H-low-high, S-low-high, V-low-high
int thresh = 100;
int Hue[] {50, 205};
int Sat[] {0, 222};
int Val[] {145, 255};
int minArea = 142;//play with this
double minPerimeter = 0.0;
double minWidth = 0.0;
double maxWidth = 1000.0;
double minHeight = 0.0;
double maxHeight = 1000.0;
int solidity[] = {70, 1000};
double maxVertices = 1000000.0;
double minVertices = 0;//play with this
int minRatio = 16;//play with this, should be more than .25, less than 1 (each increment is a hundredth)
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

void ReduceBrightness(Mat &input, Mat &output, int factor);
void HSVThreshold(Mat &input, Mat &output,int Hue[],int Sat[], int Val[]);//created above main so it can be used, but defined later to keep from using up space
void GetContours(Mat &input, ShapeArray &contours, Mat &Output);
void FilterContours(ShapeArray &input, ShapeArray &output, Mat &OutputImage);
void GenerateTargetReport(ShapeArray &input, Report Goal[], Mat &FilteredGoals, string &DataWord);
void SendRobotDataUDP();
void thresh_callback(int, void* );
void AVGSample(int count,int raw, int &avg);




int main()
{

//establish socket
//	int server, client, valread, c;
//	struct sockaddr_in address;
//	char buffer[1024] = {0};
//	if ((server = socket(AF_INET, SOCK_STREAM, 0)) == 0) { perror("socket failed"); /*exit(EXIT_FAILURE);*/ }
//	address.sin_family = AF_INET;
//	address.sin_addr.s_addr = INADDR_ANY;
//	address.sin_port = htons( PORT );
//	if (bind(server, (struct sockaddr *)&address, sizeof(address))<0) { perror("bind failed"); /*exit(EXIT_FAILURE);*/ }
//	if (listen(server, 3) < 0){ perror("listen"); /*exit(EXIT_FAILURE);*/ }
//	if ((client = accept(server, (struct sockaddr *)&address, (socklen_t*) &c  ))<0) { perror("accept"); /*exit(EXIT_FAILURE);*/ }
//	valread = recv( client , buffer, 1024,0);
//create videostream object and image
// from stream
	VideoCapture Stream;
	Stream = VideoCapture(0);//starts a camera stream
	if(Stream.open(0)== false)
	{
		printf("unable to open stream");
		exit(EXIT_FAILURE);
	}

	Stream.set(CV_CAP_PROP_FRAME_WIDTH, X_IMAGE_RES);
	Stream.set(CV_CAP_PROP_FRAME_HEIGHT, Y_IMAGE_RES);
	//from image
	Mat Original(X_IMAGE_RES, Y_IMAGE_RES, CV_CN_MAX);
	Original= imread(OriginalImage);// loads an image from the file
	int FPSWait=20;
//begin program
	while(true)
	{

		string DataWord;
		int64 start = cv::getTickCount();
	//Grab Image
		//printf("Final Camera ID %i/n", CAM_ID);
		Mat frame(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC3);// create a matrix of pixil data called frame THAT MATCHES CAMERA RESOLUTION
		Stream.retrieve(frame);// from usb, grab a frame if new frame is available
		if(Stream.read(frame)==false)
		{
			printf("unable to connect to camera");
			exit(EXIT_FAILURE);
		}
		imshow("Captured image", frame);

		//frame= Original;// from filesystem
		//HSV filter
		Mat HSVThresholdOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC3);// 1 channel binary
		HSVThreshold(frame, HSVThresholdOutput,Hue,Sat,Val);// each step of the process should be like this
	//identify contours
		ShapeArray contours; //created empty vector
		Mat ContoursOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC1);
		GetContours(HSVThresholdOutput, contours,ContoursOutput);
	//Filter contours

		ShapeArray Goals;
		FilterContours(contours, Goals, frame);// returning image of different size?
//calculate and score object
		Report GoalReport[Goals.size()];
		Mat TargetOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC1);
		GenerateTargetReport(Goals, GoalReport, frame, DataWord);
		//imshow("TargetOutput", TargetOutput);

















//		//provide the user an interface=
//		int Rawfps =rint(cv::getTickFrequency() / (cv::getTickCount() - start));
//		string Sfps= "FPS:" + std::to_string(Rawfps);
//		cv::putText(frame, Sfps, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.4, cvScalar(200,200,250), 1, CV_AA);
		imshow("Final Image", frame);
//		imshow("Binary Mask", ContoursOutput);
//		imshow("HSV", HSVThresholdOutput);
//	// Create Window
//		int max_thresh = 255;
//		char source_window[] = "Source";
//		namedWindow( source_window, CV_WINDOW_AUTOSIZE );
//	//Create Trackbars
//		createTrackbar( " MaxHue:", "Source", &Hue[1], max_thresh);
//		createTrackbar( " MinHue:", "Source", &Hue[0], max_thresh);
//		createTrackbar( " MaxSat:", "Source", &Sat[1], max_thresh);
//		createTrackbar( " MinSat:", "Source", &Sat[0], max_thresh);
//		createTrackbar( " MaxVal:", "Source", &Val[1], max_thresh);
//		createTrackbar( " MinVal:", "Source", &Val[0], max_thresh);
//		createTrackbar( " FPSWait:", "Source", &FPSWait, 250);
//		createTrackbar( " MinArea:", "Source", &minArea, 5000);
//		createTrackbar( " MinRatio:", "Source", &minRatio, 200);
//		createTrackbar( " MinSolid:", "Source", &solidity[0], 1000);
		waitKey(FPSWait);// waits to exit program until we have pressed a key
//	//build word to send
//		char *data = new char[DataWord.length() + 1];
//		strcpy(data, DataWord.c_str());
//		printf("%c",data);
//	//send word over network
//		send(client , data , strlen(data) , 0 );
	}
	return 0;
}





void ReduceBrightness(Mat &input, Mat &output, int factor)
{
//Mat new_image = Mat::zeros( input.size(), input.type() );
	for(int i = 1; i < X_IMAGE_RES-1; i++)
	{
		usleep(1000);
		for(int j = 1; j < Y_IMAGE_RES-1; j++)
		{
			for(int c=0; c < 3; c++)
			{
				//int value = int(input.at<Vec3i>(j,i)[c]);
				//value = value*float(factor);
				//cout << value << endl;
				input.at<Vec3i>(j,i)[c] = 0;
				printf("%i,%i,%i \n",i, j, c );

			}

		}

	}
	printf("--------------------------------------------------------------------------------------------------------------------\n");
}

void HSVThreshold(Mat &input, Mat &output, int Hue[], int Sat[], int Val[])// this function uses an input and output givin by the user allowing major flexibility
{
	cvtColor(input,output,COLOR_BGR2HSV);// convert the image from color image channles to 2 channel HSV "black white", binary
	inRange(output, Scalar(Hue[0],Sat[0],Val[0]),Scalar(Hue[1],Sat[1],Val[1]),output);
	imwrite(HSVfilteredImage,output);// create a new image to show the processing worked
}

void GetContours(Mat &input, ShapeArray &contours, Mat &output)
{
	Mat CannyOutput;
	Canny(input, CannyOutput, X_IMAGE_RES, Y_IMAGE_RES);
	vector<Vec4i> hierarchy;
	findContours(CannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	imwrite(ContoursImage, CannyOutput);
	output=CannyOutput;
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
				if (ratio < (minRatio/100) || ratio > maxRatio) continue;
				output.push_back(store);
				store.clear();

			}
			drawContours(OutputImage, hull, -1, 255, CV_FILLED);
			imwrite(filteredImage, OutputImage);
}

void GenerateTargetReport(ShapeArray &input, Report Goal[], Mat &FilteredGoals, string &DataWord)
{
	int sizeOfArray=int(input.size());// cast to signed int and subtract 1 to prevent using a number outside the scope of the array
	if(sizeOfArray==0)
	{
		//printf("0 goal(s), \n");
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
		printf("%i goal(s), Goal[%i] POI is:[%f], Distance is:[%f] \n", Goal[i].GoalCount,i+1, Goal[i].PointOfAim, Goal[i].Distance);


	}
}

void SendRobotDataUDP(){

}

void thresh_callback(int, void* )
{

}
void AVGSample(int count,int raw, int &avg)
{
	int sample[]{};
		int Average;
		for(int i; i<count; i++){
			sample[i]=raw;
			Average =Average+sample[i];
		}
		avg=Average;
}
