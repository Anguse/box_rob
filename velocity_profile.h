/*
 * velocity_profile.h
 *
 *  Created on: 20 maj 2019
 *      Author: harald
 */
#include <eigen3/Eigen/Dense>


#ifndef VELOCITY_PROFILE_H_
#define VELOCITY_PROFILE_H_

using Eigen::VectorXd;

VectorXd generateEncoderValues(double travelDistanceMM, int gear);


#endif /* VELOCITY_PROFILE_H_ */
