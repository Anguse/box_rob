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
#include "constants.h"

using namespace std;
using Eigen::VectorXd;
static const int CHANNEL = 1;


VectorXd generateEncoderValues(double travelDistanceMM){

	VectorXd positions(1);
	// Setup

	double maxAcc = 10000.0*2;
	double maxDeAcc = -10000.0*2;
	double maxSpeed = 2000.0*8;
	double startPos = 0.0;
	double endPos = travelDistanceMM*PULSES_PER_MM;
	double breakPos = 0.0;
	double pos = startPos;
	double speed = 0.0;
	double oldSpeed = 0.0;
	int i = 0;

	if(endPos > startPos){
		cout << "pulses per mm: " << PULSES_PER_MM << "\n";
		cout << "travel distance in mm: " << travelDistanceMM << "\n";
		cout << "endpos: " << endPos << "\n";
		while(endPos > pos && speed >= 0){
			if(speed < maxSpeed && pos < (endPos + breakPos)){
				speed = oldSpeed + maxAcc*SAMPLE_TIME;
				breakPos = speed*speed/(2*maxDeAcc);
			}else if(pos > (endPos + breakPos)){
				speed = oldSpeed + maxDeAcc*SAMPLE_TIME;
			}else{
				speed = maxSpeed;
			}
			if(positions.size()==i){
				positions.conservativeResize(positions.size()+1);
			}
			pos += speed*SAMPLE_TIME;
			positions(i) = pos;
			oldSpeed = speed;
			i++;
		}
	}
	return positions;
}
