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


#endif /* CONSTANTS_H_ */
