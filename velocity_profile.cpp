/*********  compile and link with command *********
g++ spitest.cpp -lwiringPi -o spitest
**********  run with  *****************************
./spitest
********** break the program with *****************
control-c
***************************************************/

#include <iostream>
#include <errno.h>
#include <unistd.h>
#include "spi_com.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include "velocity_profile.h"

using namespace std;
using Eigen::VectorXd;
static const int CHANNEL = 1;


VectorXd generateEncoderValues(double travelDistanceMM){

	VectorXd positions(1);
	// Setup
	int encoder = 128;
	int encoderCounter = 4;
	double h = 0.01;
	double gearReduction = 75/19;
	double gearBoxRatio = 24/1*1/2.5;
	double wheelCircumference = 42*M_PI;
	double nbrPulsePerRevolution = encoder*encoderCounter*gearReduction;
	double nbrPulsePerMM = nbrPulsePerRevolution*gearBoxRatio/wheelCircumference;

	double maxAcc = 10000*8*8;
	double maxDeAcc = -10000*8*8;
	double maxSpeed = 4000*2*2;
	double startPos = 0;
	double endPos = travelDistanceMM*nbrPulsePerMM;
	double breakPos = 0;
	double pos = startPos;
	double speed = 0;
	double oldSpeed = 0;
	int i = 0;

	if(endPos > startPos){
		while(endPos > pos && speed >= 0){
			if(speed < maxSpeed && pos < (endPos + breakPos)){
				speed = oldSpeed + maxAcc*h;
				breakPos = speed*speed/(2*maxDeAcc);
			}else if(pos > (endPos + breakPos)){
				speed = oldSpeed + maxDeAcc*h;
			}else{
				speed = maxSpeed;
			}
			if(positions.size()==i){
				positions.conservativeResize(positions.size()+1);
			}
			pos += speed*h;
			positions(i) = pos;
			oldSpeed = speed;
			i++;
		}
	}
	return positions;
}
