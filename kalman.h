/*
 * Kalman.h
 *
 *  Created on: 14 aug. 2019
 *      Author: harald
 */

#include <eigen3/Eigen/Dense>

#ifndef KALMAN_H_
#define KALMAN_H_

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd kalman_filter(MatrixXd cox_var, MatrixXd odometry_var, VectorXd cox_pos, VectorXd odometry_pos);

#endif /* KALMAN_H_ */
