/*
 * lidar.h
 *
 *  Created on: 23 maj 2019
 *      Author: harald
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

using Eigen::VectorXd;
using Eigen::MatrixXd;

int lidarInit(const char *filepath);
int lidarStop();
int lidarCoxStart(VectorXd currentPosXYA, bool RFF);
bool lidarCoxDone();
VectorXd lidarGetCoxAdj();
MatrixXd lidarGetVariance();
void lidarSetVariance(MatrixXd newVariance);

#endif /* LIDAR_H_ */
