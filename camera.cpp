

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
#include <pthread.h>

#include "camera.h"

using namespace std;
using namespace cv;

int box_status = -1;

pthread_mutex_t lock;


//initial min and max HSV filter values. Commented worked best on robot.
int H_MIN = 99;				// 99
int H_MAX = 256;			// 256
int S_MIN = 150;			//100
int S_MAX = 256;			//256
int V_MIN = 32;				//32
int V_MAX = 219;			//219
int boxNumArea;
int boxNumHier;
double DIST2BOX = 5000;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//The distance from center which will decide how large the frame where differentiating between ones and zeroes will be.
int from_cen = 80;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;  //Originally 50
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
	if (hierarchy.size() > 2)
		return 0;
	else
		return 1;
}

//Will return an int, which will be the same number as the box. If there's no indication of what number the box has the function return -1. 
// If the box is considered to be a zero it returns 0. If the box is considered to be a one it returns 1.
int trackFilteredObject(int &middlePointX, int &middlePointY, Mat threshold, Mat &cameraFeed) {

	Mat temp;
	threshold.copyTo(temp);

	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	Mat canny_output;
	Canny(temp, canny_output, thresh, thresh * 2, 3);

	findContours(canny_output, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

	/*//This is to draw the contour if the frame. Will not be used on robot.
	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)
	{
		Scalar color = Scalar(0, 255, 0);
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		Scalar color2 = Scalar(255, 0, 0);

	}


	/// Show in a window
	namedWindow("Contours", WINDOW_AUTOSIZE);
	imshow("Contours", drawing);
	*/

	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;

	if (hierarchy.size() > 0) {
		//Go for biggest area contour ie closest box.
		int biggestContAreaID = getMaxAreaContourId(contours);
		if(biggestContAreaID == -1){
			cout << "vector subscript out of range" << endl;
			return -1;
		}
		Moments middleMoment = moments((cv::Mat)contours[biggestContAreaID]);
		double area = middleMoment.m00;

		//if the area is less than 20 px by 20px then it is probably just noise
		//if the area is the same as the 3/2 of the image size, probably just a bad filter
		//we only want the object with the largest area so we safe a reference area each
		//iteration and compare it to the area in the next iteration.
		if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA) {
			middlePointX = middleMoment.m10 / area;
			middlePointY = middleMoment.m01 / area;
		}
		int middlePointXCamera = int(FRAME_WIDTH / 2);
		int middlePointYCamera = int(FRAME_HEIGHT / 2);

		int distFromCen = middlePointXCamera - middlePointX;
		int distCamera2Obj = FRAME_HEIGHT - middlePointY;

		if (middlePointX < FRAME_WIDTH / 2 + from_cen && middlePointX > FRAME_WIDTH / 2 - from_cen ) {
			if (contourArea(contours.at(biggestContAreaID)) > DIST2BOX) {
				boxNumArea = evalBoxArea(canny_output, contours);
				boxNumHier = evalBoxHier(hierarchy);
				return 1;
			}else
				return 4; //Centered object found but not close enough to classify
		}
		else if (middlePointX > middlePointXCamera + from_cen) {
			//cout << "The object is on the right side" << endl;
			return 2;
		}
		else {
			//cout << "The object is on the left side" << endl;
			return 3;
		}
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
	Mat increasedFeed;
	Mat cameraFeed;
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
			
			pthread_mutex_lock(&lock);
			box_status = numFromBox;
			pthread_mutex_unlock(&lock);

			if (numFromBox == 0)
				cout << "It's a zero" << endl;
			if (numFromBox == 1)
				cout << "It's a one!" << endl;
			if (numFromBox == 2)
				cout << "Not sure what it is, turn RIGHT until numFromBox is either 1 or 0!" << endl;
			if (numFromBox == 3)
				cout << "Not sure what it is, turn LEFT until numFromBox is either 1 or 0!" << endl;
			if (numFromBox == 4){
				cout << "Found centered box but far away. Go forward.\n" << endl;
			}
		}
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(30);
	}
	
	return 0;
}

void *camera_thread_start(void *arg){

	pthread_mutex_init(&lock, NULL);

	pthread_setcancelstate(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

	int success = cameraFindBox();

}

int cameraGetBoxStatus(){

	pthread_mutex_lock(&lock);
	int old_status = box_status;
	box_status = -1;
	pthread_mutex_unlock(&lock);

	return old_status;
}

void cameraSetBoxStatus(int newStatus){

	pthread_mutex_lock(&lock);
	box_status = newStatus;
	pthread_mutex_unlock(&lock);
}

