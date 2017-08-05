#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>
//opencv includes
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

/*
 * 1080Vision.cpp
 *	Author: 1080 Resurgence Robotics
 *  Created on: Jul 30, 2017
 */
// functions to override
//Camera constants used for distance calculation
#define X_IMAGE_RES 320		//X Image resolution in pixels, should be 160, 320 or 640
#define Y_IMAGE_RES 640
#define VIEW_ANGLEH 61		//MICROSOFT LIFECAM HD-5000
#define VIEW_ANGLEV - 34.3
//#define VIEW_ANGLE 43.5  //Axis M1011 camera -- OUR IP CAM FROM KOP

//CLASSIFICATION OF THE OBJECT WE ARE PROCESSING used for target identification *** unit = pixel ***
//#define ASPECT_RATIO 2 // FOR RECTANGULAR OBJECTS only
#define X_EDGE_LIMIT 3
#define Y_EDGE_LIMIT 4

//Minimum area of particles to be considered
#define AREA_MINIMUM 500// 500 pixels
// define the HSV threshold values for filtering our target
int HSV[6] { 60, 100, 90, 255, 20, 255};	//HSV threshold criteria array, ranges are in that order H-low-high, S-low-high, V-low-high
//Edge profile constants of our shape
#define XMAXSIZE 300
#define XMINSIZE 24
#define YMAXSIZE 600
#define YMINSIZE 24
//Structure to represent the scores for the various tests used for target identification WILL BE USED LATER
	struct Scores {
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;

	};

	//TODO: start a camera stream and display an image from the while loop
int main()
{
	//create vcideostream object
	// create windows for viewing each step in filter process until we are able to run without them ie... "calibrated"

	while(true)//change later to something like "while an image exists"
	{

	//grab one frame from camera

	//process it-> WHEN EACH OF THE BELOW STEPS IS COMPLETE THEY SHOULD HAVE A CORRESPONDING WINDOW FOR PREVIEWS.
		//HSV filter
		printf("Threshold: %i, %i, %i, %i, %i, %i, \n", HSV[0], HSV[1], HSV[2], HSV[3], HSV[4], HSV[5]  );// how to use arrays
		//canny edge detect edges within the object
		//convex hull the image
		//remove any smaller particles
		//perform algorithms described later to score and classify the object
	}

	return 0;
}



