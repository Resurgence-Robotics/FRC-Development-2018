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

int main2()
{
    // Create a small image with a circle in it.
    Mat image(256, 256, CV_8UC3, Scalar(0, 0, 0));
    circle(image, Point(80, 110), 42, Scalar(255,127, 63), -1);

    // Find canny edges.
    Mat cannyEdges;
    Canny(image, cannyEdges, 80, 60);

    // Show the images.
    imshow("img", image);
    imshow("cannyEdges", cannyEdges);

    // Find the contours in the canny image.
    vector<Vec4i> hierarchy;
    typedef vector<vector<Point> > TContours;
    TContours contours;
    findContours(cannyEdges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // cannyEdges is destroyed after calling findContours

    // Print number of found contours.
    std::cout << "Found " << contours.size() << " contours." << std::endl;

    waitKey();
    return 0;
}
