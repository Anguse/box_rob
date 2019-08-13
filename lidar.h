/*
 * lidar.h
 *
 *  Created on: 23 maj 2019
 *      Author: harald
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <eigen3/Eigen/Dense>

using Eigen::VectorXd;

int lidarInit(const char *filepath);
int lidarStop();
int lidarCoxStart(VectorXd currentPosXYA, bool RFF);
bool lidarCoxDone();
VectorXd lidarGetCoxAdj();

#endif /* LIDAR_H_ */
