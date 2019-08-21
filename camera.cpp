

#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/cvdef.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "camera.h"

using namespace std;
using namespace cv;

bool box_found;

//initial min and max HSV filter values. Commented worked best on robot.
int H_MIN = 99;				// 99
int H_MAX = 256;			// 256
int S_MIN = 100;			//100
int S_MAX = 256;			//256
int V_MIN = 32;				//32
int V_MAX = 216;			//219
double DIST2BOX = 9000;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//The distance from center which will decide how large the frame where differentiating between ones and zeroes will be.
int from_cen = 100;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 25;  //Originally 50
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

//Go forwards as long as this is set to true as that means there is a contour of interest in middle of frame.
bool capFlag = false;
//Counter for how many one boxes the robot has collected. When ones == 2 then robot will return to home.
int ones = 0;

int thresh = 100;
int max_thresh = 255;

//Double to find optimal between height and width of contour.
double differenceHW = 0.17;
//int for while loop when looking at boxes. Set to -1 when no box in frame. If box in frame is 0 then numFromBox = 0. If box is 1 then numFromBox = 1;
int numFromBox = -1;

RNG rng(12345);

string intToString(int number) {


	std::stringstream ss;
	ss << number;
	return ss.str();
}

void morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));	//Size originally (3,3)
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(9, 9)); //Size originally (9,9)

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);


	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);

}
//Returns contour in frame with largest area.
int getMaxAreaContourId(vector <vector<cv::Point> > contours) {
	double maxArea = 0.0;
	int maxAreaContourId = -1;
	for (int j = 0; j < contours.size(); j++) {
		double newArea = contourArea(contours.at(j));
		
		if (newArea > maxArea) {
			maxArea = newArea;
			maxAreaContourId = j;
		}
	}
	return maxAreaContourId;
}

int evalBoxArea(Mat canny_output, vector <vector<cv::Point> > contours) {
	int contID = getMaxAreaContourId(contours);

	findNonZero(canny_output, contours[contID]);
	Rect boundRect = boundingRect(contours[contID]);
	
	if (boundRect.height *differenceHW < boundRect.width) {
		cout << "This is bound width: " << boundRect.width << "\n" << "This is bound height : " << boundRect.height << "\n" << endl;
		return 0;
	}
	else
	{
		cout << "This is bound width: " << boundRect.width << "\n" << "This is bound height " << boundRect.height << "\n" << endl;
		return 1;
	}

}


//Decide whether or not the cropped image contains a box with zero or one based on hierarchy
int evalBoxHier(vector<Vec4i> hierarchy) {
	if (hierarchy.size() > 2) {
		return 0;
	}
	else {
		return 1;
	}

}

//Will return an int, which will be the same number as the box. If there's no indication of what number the box has the function return -1. 
// If the box is considered to be a zero it returns 0. If the box is considered to be a one it returns 1.
int trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed) {

	Mat temp;
	threshold.copyTo(temp);

	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

	//This is to draw the contour if the frame. Will not be used on robot.
//	Mat drawing = Mat::zeros(temp.size(), CV_8UC3);
//	for (int i = 0; i < contours.size(); i++)
//	{
//		Scalar color = Scalar(0, 255, 0);
//		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
//		Scalar color2 = Scalar(255, 0, 0);
//
//	}
//
//
//	/// Show in a window
//	namedWindow("Contours", WINDOW_AUTOSIZE);
//	imshow("Contours", drawing);

	
	//use moments method to find our filtered object
	int biggestContArea = getMaxAreaContourId(contours);
	double refArea = 0;
	bool objectFound = false;
	//If the contour in question has atleast some kind of hierarchy and that the contour is close enough(based on the area of the contour).
	if (hierarchy.size() > 0 && (contourArea(contours.at(biggestContArea)) > DIST2BOX)) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects < MAX_NUM_OBJECTS) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
				}
				else objectFound = false;
			}
			//let user know you found an object
			if (objectFound == true) {

				if (x < FRAME_WIDTH / 2 + from_cen && x > FRAME_WIDTH / 2 - from_cen) {
					//cout << "Object is centered" << "\n";
					capFlag = true;

					Mat frame2, frame2_gray;
										
					frame2 = temp.clone();
					Rect myROI = Rect(FRAME_WIDTH / 2 - from_cen, 0, 100, FRAME_HEIGHT);
					Mat image_roi = frame2(myROI).clone();
					blur(image_roi, image_roi, Size(3, 3));
					//Create a canny with set threshold
					Mat canny_output;
					Canny(image_roi, canny_output, thresh, thresh * 2, 3);

					//Find contours in cropped part of frame
					vector< vector<Point> > contours2;
					vector<Vec4i> hierarchy2;
					findContours(canny_output, contours2, hierarchy2, RETR_LIST, CHAIN_APPROX_SIMPLE);
					
					int boxNumArea, boxNumHier;
					boxNumArea = evalBoxArea(canny_output, contours);
					boxNumHier = evalBoxHier(hierarchy2);
					
					//If both area and hierarchy classifies the box as a one then return 1.
					if (boxNumArea == 1 && boxNumHier == 1) {
						return 1;
					}
					else
					{
						return 0;
					}
						
				}
				else if (x > FRAME_WIDTH / 2 + from_cen) {
					//Close enough but on the positive side.
					return 2;
				}
				else {
					//Close enough but on the negative side.
					return 3;
					
				}
			}

		}
		else cout << "TOO MUCH NOISE! ADJUST FILTER";
	}
	return -1;
}

int cameraFindBox()
{
	//some boolean variables for different functionality within this
	//program
	bool trackObjects = true;
	bool useMorphOps = true;
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat increasedFeed;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage fo200r binary threshold image
	Mat threshold;
	//x and y values for the location of the object
	int x = 0, y = 0;
	//create slider bars for HSV filtering
	//createTrackbars();
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while (1) {
		
		cameraFeed.convertTo(increasedFeed, -1, 1.3, 40);

		//store image to matrix
		capture.read(increasedFeed);

		//convert frame from BGR to HSV colorspace
		cvtColor(increasedFeed, HSV, COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if (useMorphOps)
			morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		

		/*
		To collect box with one drive straight a certain time. As it is guaranteed to be at a certain distance from the robot due to it only being classified
		at a certain distance. When that time has gone, update some counter with 1 to indicate box has been collected. Drive back home, leave box. Look for new box.
		Do this until counter = 4; (Due to time constraints this will not happen).
		
		To aboid box with zero same here. It will guaranteed be at a certain distance. Therefor turn a certain angle to avoid accidentaly picking it up when going
		for another box in the frame.
		
		*/
		if (trackObjects) {
			/*
			Num from box:

			-1 = no box is found and the robot should look for a box.
			0 = Box found is classified as zero. Drive to find new box.
			1 = Box found is classified as one and centered. Drive to collect it. 
			2 = Box found is close enough to be classified but not centered. Turn to the right.
			3 = Box found is close enough to be classified but not centered. Turn to the left.
			
			*/
			numFromBox = trackFilteredObject(x, y, threshold, increasedFeed);
			
			if (numFromBox == 0)
				cout << "It's a zero" << endl;
			if (numFromBox == 1)
				cout << "It's a one!" << endl;
			if (numFromBox == 2)
				cout << "Not sure what it is, turn RIGHT until numFromBox is either 1 or 0!" << endl;
			if (numFromBox == 3)
				cout << "Not sure what it is, turn LEFT until numFromBox is either 1 or 0!" << endl;
		}
		if(numFromBox == 1){
			// Distance = 450mm
			return 1;
		}
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		//waitKey(30);
	}
	
	return 0;
}

void *camera_thread_start(void *arg){

	int success = cameraFindBox();
	if(success == 1)
		box_found = true;
	else
		box_found = false;

}

bool cameraGetBoxStatus(){

	return box_found;
}

void cameraSetBoxStatus(bool newStatus){

	box_found = newStatus;

}

