#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <errno.h>
//#include <unistd.h>
//#include "spi_com.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <Eigen/Dense>
#include <math.h>
#include "velocity_profile.h"


using namespace std;
using Eigen::VectorXd;
static const int CHANNEL = 1;

double velocityProfileOffset = 6/170;	// 10mm in 340mm, 6mm in 170


VectorXd generateEncoderValues(double travelDistanceMM) {

//	travelDistanceMM += 7;

	VectorXd positions(1);
	VectorXd speed(1);
	VectorXd acceleration(1);
	// Setup
	double dt = 0.01;									// Sample rate
	double skip_sspeed = 0;

	double start_pos = 0;
	double pos = start_pos;
	double initial_speed = 0;
	double maxSpeed = 13000;
	double acc = 20000;


	int encoder = 128;
	int encoderCounter = 4;
	double h = 0.01;
	double gearReduction = 75 / 19;
	double gearBoxRatio = 24 / 1 * 1 / 2.5;
	double wheelCircumference = 42 * M_PI;
	double nbrPulsePerRevolution = encoder * encoderCounter*gearReduction;
	double nbrPulsePerMM = nbrPulsePerRevolution * gearBoxRatio / wheelCircumference;

	double endPos = travelDistanceMM * nbrPulsePerMM;

	double d_pos = abs(endPos - pos);						//Difference in position end pos - initial position
	double d_spd = abs(maxSpeed - initial_speed);			//%Difference in speed, Maximum speed - initial speed(set to 0 in call)
	double a_t = d_spd / acc;							//%How long do we accelerate ? , Speed divided by acc to see for how long we acc.
	double a_t_discrete = a_t / dt;						//%(in ticks)  Here in ticks
	double spd_inc = d_spd / a_t_discrete;				//%Every tick, increase spd by

	//Position from acc :
	double acc_pos = 0;                    //% For each tick get position from acceleration
	for (int i = 0; i < a_t_discrete; i++) {
		acc_pos += (i*spd_inc*dt);
	}
	//It's possible to overshoot position if the acceleration is too low.
	//In that case we should sacrifice the top speed
	if ((2 * acc_pos) > d_pos) {

		maxSpeed = sqrt(acc*d_pos);

		//Redo the initial math :
		d_spd = abs(maxSpeed - initial_speed);        //Difference in speed
		a_t = d_spd / acc;                //How long do we accelerate ?
		a_t_discrete = a_t / dt;			// (in ticks)
		spd_inc = d_spd / a_t_discrete;	// Every tick, increase spd by
		//Position from acc :
		acc_pos = 0;
		for (int i = 0; i < a_t_discrete; i++) {
			acc_pos = acc_pos + (i*spd_inc*dt);
		}
	}

	double cte_spd_pos = d_pos - 2 * acc_pos;
	double cte_spd_pos_discrete = (cte_spd_pos / maxSpeed) / dt;
	if (cte_spd_pos_discrete < 0) {
		skip_sspeed = 1;
	}

	speed(0) = initial_speed;
	positions(0) = start_pos;
	acceleration(0) = acc;

	double tmp_spd = 0;
	double tmp_pos = 0;
	double tmp_acc = 0;


	//Positive acceleration
	for (int i = 1; i < a_t_discrete; i++) {
		tmp_spd = speed(i - 1) + spd_inc;
		tmp_pos = speed.sum()*dt;
		tmp_acc = acc;
		if (i == speed.size()) {
			speed.conservativeResize(speed.size() + 1);
			positions.conservativeResize(positions.size() + 1);
			acceleration.conservativeResize(acceleration.size() + 1);
		}
		speed(i) = tmp_spd;
		positions(i) = tmp_pos;
		acceleration(i) = tmp_acc;
	}
	//Constant speed
	if (skip_sspeed == 0) {
		for (int i = a_t_discrete ; i < cte_spd_pos_discrete + a_t_discrete; i++) {
			tmp_spd = abs(speed(i-1));
			tmp_pos = speed.sum()*dt;
			tmp_acc = 0;
			if (i == speed.size()) {
				speed.conservativeResize(speed.size() + 1);
				positions.conservativeResize(positions.size() + 1);
				acceleration.conservativeResize(acceleration.size() + 1);
			}
			speed(i) = tmp_spd;
			positions(i) = tmp_pos;
			acceleration(i) = tmp_acc;
		}
	}
	//Negative acceleration
	for (int i = cte_spd_pos_discrete + a_t_discrete; i < a_t_discrete*2+cte_spd_pos_discrete; i++) {
		
		tmp_spd = speed(i - 1) - spd_inc;
		double test = speed.sum();
		tmp_pos = speed.sum()*dt;
		
		tmp_acc = -acc;
		if (i == speed.size()) {
			speed.conservativeResize(speed.size() + 1);
			positions.conservativeResize(positions.size() + 1);
			acceleration.conservativeResize(acceleration.size() + 1);
		}
		speed(i) = tmp_spd;
		positions(i) = tmp_pos;
		acceleration(i) = tmp_acc;
		
	}
	return positions;
}
