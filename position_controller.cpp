/*
 * main.cpp
 *
 *  Created on: 20 maj 2019
 *      Author: haraldx
 */

#include "velocity_profile.h"
#include "lidar.h"
#include "spi_com.h"
#include <wiringPiSPI.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include "constants.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

// SPI
static const int CHANNEL = 1;
TYPE_White_Board_RX *pWhite_Board_RX;
TYPE_White_Board_TX *pWhite_Board_TX;
unsigned char buffer[100];

// Odometry
VectorXd posXYA(3);		// Current position
MatrixXd cXYA(3,3);		// Current uncertainty
bool travel_done;

// Log
ofstream pos_log;

int updatePos(VectorXd Er, VectorXd El){

	// Calculate odometry
	int n = Er.size();
	double adjustmentFactor;
	VectorXd Dr;
	VectorXd Dl;
	MatrixXd posAdjXYA(n,3);
	MatrixXd cStorage(n,9);
	MatrixXd Sx(3,3);
	MatrixXd cAdjXYA(3,3);
	MatrixXd Js(3,3);
	MatrixXd Jm(3,2);
	VectorXd Jb(3);
	MatrixXd Sm(2,2);
	double Sb = SIGMA_WB;
	double dDr, dDl, dD, dA, dX, dY;

	posAdjXYA.row(0) << posXYA(0), posXYA(1), posXYA(2);		// Adjusting 90 degrees for calculation?
	cStorage.row(0) << cXYA(0), 0, 0, 0, cXYA(1), 0, 0, 0, cXYA(2);
	Dr = Er/PULSES_PER_MM;
	Dl = El/PULSES_PER_MM;

	if(n > 1){
		for(int i = 1; i < n; i++){
			dDr = Dr(i) - Dr(i-1);
			dDl = Dl(i) - Dl(i-1);
			dD = (dDr+dDl)/2;
			dA = (dDr+dDl)/WHEELBASE_MM;

			dX = dD*cos(posAdjXYA(i-1,2)+dA/2);
			dY = dD*sin(posAdjXYA(i-1,2)+dA/2);

			posAdjXYA(i,0) = posAdjXYA(i-1,0) + (dDr+dDl)/2*sin(posAdjXYA(i-1,2)+(dDr-dDl)/(2*WHEELBASE_MM)); //Swapped sin and cos
			posAdjXYA(i,1) = posAdjXYA(i-1,1) + (dDr+dDl)/2*cos(posAdjXYA(i-1,2)+(dDr-dDl)/(2*WHEELBASE_MM));
			posAdjXYA(i,2) = fmod(posAdjXYA(i-1,2) + (dDr-dDl)/WHEELBASE_MM, 2*M_PI);

			// Log
			pos_log << posAdjXYA(i,0) << "\t" << posAdjXYA(i,1) << "\t" << posAdjXYA(i,2) << "\n";

			//Predict new uncertainty
			Sx.row(0) << cStorage(i-1,0), cStorage(i-1,1), cStorage(i-1,2);
			Sx.row(1) << cStorage(i-1,3), cStorage(i-1,4), cStorage(i-1,5);
			Sx.row(2) << cStorage(i-1,6), cStorage(i-1,7), cStorage(i-1,8);
			Sm << SIGMA_RW*abs(dDr), 0,
				  0, SIGMA_LW*abs(dDl);

			Js << 1, 0, -sin(posAdjXYA(i-1,2)-(dDl - dDr)/(2*WHEELBASE_MM))*(dDl/2 + dDr/2),
				  0, 1,  cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))*(dDl/2 + dDr/2),
				  0, 0,  1;

			Jm << cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))/2 - (sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))*(dDl/2 + dDr/2))/(2*WHEELBASE_MM), cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))/2 + (sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))*(dDl/2 + dDr/2))/(2*WHEELBASE_MM),
				  sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))/2 + (cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))*(dDl/2 + dDr/2))/(2*WHEELBASE_MM), sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))/2 - (cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))*(dDl/2 + dDr/2))/(2*WHEELBASE_MM),
				  1/WHEELBASE_MM,                                                                       															 -1/WHEELBASE_MM;

			Jb << -(sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))*(dDl - dDr)*(dDl/2 + dDr/2))/(2*pow(WHEELBASE_MM,2)),
				  (cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*WHEELBASE_MM))*(dDl - dDr)*(dDl/2 + dDr/2))/(2*pow(WHEELBASE_MM,2)),
				  (dDl - dDr)/pow(WHEELBASE_MM,2);

			cAdjXYA = Js*Sx*Js.transpose() + Jm*Sm*Jm.transpose() + Jb*Sb*Jb.transpose();
			cStorage.row(i) << cAdjXYA(0,0), cAdjXYA(0,1), cAdjXYA(0,2),
							   cAdjXYA(1,0), cAdjXYA(1,1), cAdjXYA(1,2),
							   cAdjXYA(2,0), cAdjXYA(2,1), cAdjXYA(2,2);
		}
		cXYA = cAdjXYA;
		posXYA = posAdjXYA.row(posAdjXYA.rows()-1);
	}
//	pos_log << "###Movement values in mm###" << endl;
//	pos_log << "X\tY\tA\t\tsX\tsY\tsA" << endl;
//	for(int i = 0; i < n; i++){
//	   pos_log << posAdjXYA(i,0) << "\t" << posAdjXYA(i,1) << "\t" << posAdjXYA(i,2) << "\t\t" << cStorage(i,0) << "\t" << cStorage(i,4) << "\t" << cStorage(i,8) << endl;
//	}
//	pos_log << "---END---" << endl;
	return 0;
}

int sendEncoderValues(VectorXd Er, VectorXd El, bool update_pos){

   int fd, result;
   int Done = 0;
   int toggle = 1;
   int startPosL, startPosR;
   int desPosL, desPosR;

   result = wiringPiSPIDataRW(CHANNEL, buffer, 32);
   startPosL = pWhite_Board_RX->Position_M0;
   startPosR = -pWhite_Board_RX->Position_M1;

   cout << "On the move...\n" << endl ;
   for(int i = 0; i < El.size(); i++){
	   desPosR = -(Er(i) + startPosR);
	   desPosL = El(i) + startPosL;
	   pWhite_Board_TX->Status = 1;
	   pWhite_Board_TX->Set_Speed_M0 = 0.1*(desPosL - pWhite_Board_RX->Position_M0);
	   pWhite_Board_TX->Set_Speed_M1 = 0.1*(desPosR - pWhite_Board_RX->Position_M1);
	   if(pWhite_Board_RX->Digital_In & 0x01 == 0x01)
		   pWhite_Board_TX->Digital_Out = 0x01;
	   else
		   pWhite_Board_TX->Digital_Out = 0x00;
	   //pWhite_Board_TX->Position_Servo = Counter & 0xff;
	   pWhite_Board_TX->Heart_Beat = 1;
	   memcpy(buffer,pWhite_Board_TX,32);
	   result = wiringPiSPIDataRW(CHANNEL, buffer, 32);
	   if(update_pos && i < El.size()-1){
		   updatePos(El.segment(i, 2), Er.segment(i, 2)); //Swapped El & Er
	   }
	   usleep(10000); // sleep in micro sek
   }
//   cout << "Done!, endpos = (" << pWhite_Board_RX->Position_M0 << ",\t" << pWhite_Board_RX->Position_M1 << ")"<< endl;
   return 0;
}

int resetEncoders(){

	int home0 = 0;
	int home1 = 0;
	int dummy0 = 0;
	int dummy1 = 0;
	int fd, result;

	pWhite_Board_TX->Status = 0;
	pWhite_Board_TX->Control = 0;
	pWhite_Board_TX->Current_Loop = 0;
	pWhite_Board_TX->Speed_Loop = 0;
	pWhite_Board_TX->Set_PWM_M0 = 0;
	pWhite_Board_TX->Set_PWM_M1 = 0;
	pWhite_Board_TX->Set_Speed_M0 = 0;
	pWhite_Board_TX->Set_Speed_M1 = 0;
	pWhite_Board_TX->Digital_Out = 0;
	pWhite_Board_TX->Spare1 = 0;
	pWhite_Board_TX->P_Value_Current = 0.0;
	pWhite_Board_TX->P_Value_Speed = 0.0;
	pWhite_Board_TX->I_Value_Speed = 0.0;
	pWhite_Board_TX->Speed_Stepper = 0;
	pWhite_Board_TX->Position_Servo = 0;
	pWhite_Board_TX->Heart_Beat = 0;

	fd = wiringPiSPISetup(CHANNEL, 4000000);

	pWhite_Board_TX->Status = 1;
	pWhite_Board_TX->Control = 1;
	pWhite_Board_TX->Set_Speed_M0 = 0;
	pWhite_Board_TX->Set_Speed_M1 = 0;
	pWhite_Board_TX->Position_Servo = 0;
	pWhite_Board_TX->Heart_Beat = 1;

	memcpy(buffer,pWhite_Board_TX,32);
	result = wiringPiSPIDataRW(CHANNEL, buffer, 32);

	cout << "Resetting encoders...\n" << endl ;
	while(!(home0&&home1)){
	   if(pWhite_Board_RX->Position_M0 < 5 && pWhite_Board_RX->Position_M0 > -5){
		   home0 = 1;
	   }
	   if(pWhite_Board_RX->Position_M1 < 5 && pWhite_Board_RX->Position_M1 > -5){
		   home1 = 1;
	   }
		dummy0 = -pWhite_Board_RX->Position_M0;
		dummy1 = -pWhite_Board_RX->Position_M1;

		if(home0){
			dummy0 = 0;
		}else if(dummy0 > 60)
			dummy0 = 60;
		else if(dummy0 < -60)
			dummy0 = -60;

		if(home1){
			dummy1 = 0;
		}else if(dummy1 > 60)
			dummy1 = 60;
		else if(dummy1 < -60)
			dummy1 = -60;

		pWhite_Board_TX->Status = 1;
		pWhite_Board_TX->Set_Speed_M0 = dummy0;
		pWhite_Board_TX->Set_Speed_M1 = dummy1;
		pWhite_Board_TX->Heart_Beat = 1;
		memcpy(buffer,pWhite_Board_TX,32);
		result = wiringPiSPIDataRW(CHANNEL, buffer, 32);
		usleep(10000); // sleep in micro sek
	  }
	cout << "Done!\n";
}

int controllerTravel(VectorXd endXYA){

	VectorXd El1;	// Part1, 2 and 3 of encoder values
	VectorXd Er1;
	VectorXd El2;
	VectorXd Er2;
	VectorXd El3;
	VectorXd Er3;

	// Convert dX, dY and dA to dD and dA
	double dX = endXYA(0) - posXYA(0);
	double dY = endXYA(1) - posXYA(1);
	if(dY == 0){
		dY = 0.000000000000000000000000001;
	}
	double dA = fmod(atan2(dX,dY)-posXYA(2),2*M_PI); //atan(y/x)
	cout << "dA = " << dA << "\n";
	double dD = sqrt(pow(dX,2)+pow(dY,2));
	cout << "dD = " << dD << "\n";

	// Rotate so there is a straight line from start to end
	if(dA > 0 && abs(dA)>M_PI/64){
		// Turn right
		cout << "turn right in the start\n";
		double dDl = abs(WHEELBASE_MM*dA/2);
		El1 = generateEncoderValues(dDl);
		Er1 = -El1;
		sendEncoderValues(Er1, El1, true);
		// Turn adjustment, not recorded in odometry
		El1 = generateEncoderValues(dDl*TURN_ADJUSTMENT_FACTOR);
		Er1 = -El1;
		sendEncoderValues(Er1, El1, false);
	}else if(dA < 0 && abs(dA)>M_PI/64){
		// Turn left
		cout << "turn left in the start\n";
		double dDr = abs(WHEELBASE_MM*dA/2);
		Er1 = generateEncoderValues(dDr);
		El1 = -Er1;
		sendEncoderValues(Er1, El1, true);
		// Turn adjustment, not recorded in odometry
		Er1 = generateEncoderValues(dDr*TURN_ADJUSTMENT_FACTOR);
		El1 = -Er1;
		sendEncoderValues(Er1, El1, false);
	}
	// Travel distance dD
	El2 = generateEncoderValues(dD);
	Er2 = El2;
	sendEncoderValues(Er2, El2, true);

//	 Rotate to desired angle
//	dA = endXYA(2) - dA;
//	if(dA > 0){
//		// Turn right
//		cout << "turn right in the end\n";
//		double dDl = abs(wheelBaseMM*dA);
//		El3 = generateEncoderValues(dDl*turnAdjustmentFactor);
//		Er3 = -El3;
//		sendEncoderValues(Er3, El3);
//	}else{
//		// Turn left
//		cout << "turn left in the end\n";
//		double dDr = abs(wheelBaseMM*dA);
//		Er3 = generateEncoderValues(dDr*turnAdjustmentFactor);
//		El3 = -Er3;
//		sendEncoderValues(Er3, El3);
//	}
	return 0;
}

int controllerInit(VectorXd startXYA){
	pos_log.open("logs/pos_controller/pos_log.txt", ofstream::out | ofstream::trunc);
	pWhite_Board_RX = (TYPE_White_Board_RX *)buffer;
	pWhite_Board_TX = (TYPE_White_Board_TX *)malloc(sizeof(TYPE_White_Board_TX));
	if(!pWhite_Board_TX){
		cout << "Couldn't allocate" << endl;
		exit(1);
	}
	cXYA << 1, 0, 0,
			0, 1, 0,
			0, 0, pow(M_PI/180,2);
	posXYA = startXYA;
	resetEncoders();
	return 0;
}

VectorXd controllerGetPos(){
	return posXYA;
}

VectorXd controllerGetC(){
	return cXYA;
}

void *travel_thread_start(void *arg){

	VectorXd endXYA(3);
	endXYA = *(VectorXd*)arg;


	controllerTravel(endXYA);


	travel_done = true;

	return NULL;
}

int main(){
	VectorXd startXYA(3), endXYA(3), midXYA(3), coxAdjXYA(3);
	pthread_t travel;
	coxAdjXYA << 0, 0, 0;
	startXYA << 1215, 250, 0;
	void *data_ptr;
	travel_done = false;

	lidarInit(NULL);
	controllerInit(startXYA);

	//####### Phase 1
	endXYA << 1215, 2000, 0;
	data_ptr = &endXYA;
	pthread_create(&travel, NULL, travel_thread_start, data_ptr);
	lidarCoxStart(posXYA+coxAdjXYA, false);
	while(!travel_done){
		if(lidarCoxDone()){
			coxAdjXYA -= lidarGetCoxAdj();
			lidarCoxStart(posXYA+coxAdjXYA, false);
		}
		usleep(100);
	}
	travel_done = false;
	endXYA = posXYA+coxAdjXYA;
	pthread_create(&travel, NULL, travel_thread_start, data_ptr);
	while(!travel_done)
		usleep(100);
	coxAdjXYA << 0, 0, 0;

	//####### Phase 2
	travel_done = false;
	data_ptr = &startXYA;
	pthread_create(&travel, NULL, travel_thread_start, data_ptr);
	lidarCoxStart(posXYA+coxAdjXYA, false);
	while(!travel_done){
		while(!lidarCoxDone()){
			usleep(100);
		}
		coxAdjXYA -= lidarGetCoxAdj();
		lidarCoxStart(posXYA+coxAdjXYA, false);
		usleep(100);
	}
	travel_done = false;
	startXYA = posXYA+coxAdjXYA;
	pthread_create(&travel, NULL, travel_thread_start, data_ptr);
	while(!travel_done)
		usleep(100);
	travel_done = false;
//	double dx = endXYA(0) - posXYA(0);
//	double dy = endXYA(1) - posXYA(1);
//	double da = fmod(atan2(dx,dy)-posXYA(2),2*M_PI);
//	double a = endXYA(2);
//
//
//	for(int i = 0; i < 10; i++){
//		endXYA << posXYA(0)+dx/10, posXYA(1)+dy/10, da;
//		pthread_create(&travel, NULL, travel_thread_start, (void*)endXYA);
//		controllerTravel(endXYA+coxAdjXYA);
//		if(lidarCoxDone()){
//			coxAdjXYA = lidarGetCoxAdj();
//			lidarCoxStart(posXYA, false);
//		}else{
//			coxAdjXYA << 0, 0, 0;
//		}
//	}
//
//	endXYA << 1215, 250, 0;
//	dx = endXYA(0) - posXYA(0);
//	dy = endXYA(1) - posXYA(1);
//	da = fmod(atan2(dx,dy)-posXYA(2),2*M_PI);
//	a = endXYA(2);
//	lidarCoxStart(posXYA, false);
//
//	for(int i = 0; i < 10; i++){
//		endXYA << posXYA(0)+dx/10, posXYA(1)+dy/10, da;
//		controllerTravel(endXYA+coxAdjXYA);
//		if(lidarCoxDone()){
//			coxAdjXYA = lidarGetCoxAdj();
//			lidarCoxStart(posXYA, false);
//		}else{
//			coxAdjXYA << 0, 0, 0;
//		}
//	}
//
//	endXYA << posXYA(0), posXYA(1), a;
//	controllerTravel(endXYA);
	pos_log.close();
	lidarStop();
	return 0;
}

