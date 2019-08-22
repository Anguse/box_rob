/*
 * camera.h
 *
 *  Created on: 21 aug. 2019
 *      Author: harald
 */

#ifndef CAMERA_H_
#define CAMERA_H_


int cameraFindBox();
void *camera_thread_start(void *arg);
int cameraGetBoxStatus();
void cameraSetBoxStatus(int newStatus);

#endif /* CAMERA_H_ */
