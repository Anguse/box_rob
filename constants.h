/*
 * kinematics.h
 *
 *  Created on: 13 aug. 2019
 *      Author: harald
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <math.h>

/*####################
 * ### ROBOT SPECS ###
 * ###################*/
const int ENCODER = 128;
const int ENCODER_COUNTER = 4;
const double SAMPLE_TIME = 0.01;
const double GEAR_REDUCTION = 75.0/19.0;
const double GEARBOX_RATIO = (24.0/1.0)*(1.0/2.5);
const double WHEEL_CIRC = 42.0*M_PI;
const double WHEELBASE_MM = 175.0;
const double PULSES_PER_REVOLUTION = ENCODER*ENCODER_COUNTER*GEAR_REDUCTION;  //
const double MM_PER_PULSE = WHEEL_CIRC/(PULSES_PER_REVOLUTION*GEARBOX_RATIO); //
const double PULSES_PER_MM = PULSES_PER_REVOLUTION*GEARBOX_RATIO/WHEEL_CIRC;  // 147

const double TURN_ADJUSTMENT_FACTOR = 0.1;	// Factor to counter slippage caused by the coaster wheel TODO adjust odometry wrt. this (58mm)
const double SIGMA_ENCODER = 0.5/12;  			// TODO: value used from intelligent vehicles, find a more suitable value?
const double SIGMA_WB = 13.0;				// TODO: used from intelligent vehicles, find a more accurate uncertainty for wheel base?
const double SIGMA_LW = SIGMA_ENCODER;			// Uncertainty in each wheel
const double SIGMA_RW = SIGMA_ENCODER;
const double SIGMA_DD = (SIGMA_LW+SIGMA_RW)/4.0;
const double SIGMA_DA = (SIGMA_LW+SIGMA_RW)/pow(WHEELBASE_MM,2);

/*######################
 * ## SENSOR MOUNTING ##
  ######################*/
const double ALPHA = 0.0;					// X-displacement
const double BETA = 0.0;					// Y-displacement
const double GAMMA = M_PI/2;	// angular displacement


/*#####################
 * ##### LIDAR ########
 * ####################*/
const double SCAN_ANGLE_ADJUSTMENT = -M_PI/180;    // Adjustment angle to radians and invert since rotation of lidar is clockwise
#define MIN_SCANS 500
#define SCAN_TIMEOUT_MS 10000
#define RECEIVE_PORT 9888
#define SEND_PORT 9887
#define HOST "127.0.0.1"

/*####################
 *####### COX ########
 *#################### */
const int MAX_ITERATIONS = 20;				// Maximum iterations
const int MIN_INLIERS = 10;		// Minimum requiered inliers
const int MAX_INLIERS = 200;
const double INLIER_THRESHOLD = 50;			// Maximum inlier distance
const double CONVERGENCE_THRESHOLD = 5;		// Treshold distance for convergence
const double CONVERGENCE_ANGULAR_THRESHOLD = 0.1*M_PI/180;
const double DISPLACEMENT_LIMIT = 150;		// Maximum allowed displacement distance
const double ANGLE_CHANGE_LIMIT = M_PI/4;

#endif /* CONSTANTS_H_ */
