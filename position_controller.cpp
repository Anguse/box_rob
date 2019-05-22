/*
 * main.cpp
 *
 *  Created on: 20 maj 2019
 *      Author: haraldx
 */

#include "velocity_profile.h"
#include "spi_com.h"
#include <wiringPiSPI.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

/*####################
 * ### ROBOT SPECS ###
 * ###################*/
int encoder = 128;
int encoderCounter = 4;
double gearReduction = 75.0/19.0;
double gearBoxRatio = 24/1*1/2.5;
double wheelCirc = 42*M_PI;
double wheelBaseMM = 170;
double pulsesPerRev = encoder*encoderCounter*gearReduction;
double mmPerPulse = wheelCirc/(pulsesPerRev*gearBoxRatio);


double turnAdjustmentFactor = 0.58;	// Factor to counter slippage caused by the coaster wheel TODO adjust odometry wrt. this
double sigmaEnc = 0.5/12;  			// TODO: value used from intelligent vehicles, find a more suitable value?
double sigmaWb = 13.0;				// TODO: used from intelligent vehicles, find a more accurate uncertainty for wheel base?
double sigmaL = sigmaEnc;			// Uncertainty in each wheel
double sigmaR = sigmaEnc;
double sigmaDd = (sigmaL+sigmaR)/4.0;
double sigmaDa = (sigmaL+sigmaR)/pow(wheelBaseMM,2);
/*#####################
 * ####################
 * ####################*/

// SPI
static const int CHANNEL = 1;
TYPE_White_Board_RX *pWhite_Board_RX;
TYPE_White_Board_TX *pWhite_Board_TX;
unsigned char buffer[100];

// Odometry
VectorXd posXYA(3);		// Current position
MatrixXd cXYA(3,3);		// Current uncertainty

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
	double Sb = sigmaWb;
	double dDr, dDl, dD, dA, dX, dY;

	posAdjXYA.row(0) << posXYA(0), posXYA(1), posXYA(2);		// Adjusting 90 degrees for calculation?
	cStorage.row(0) << cXYA(0), 0, 0, 0, cXYA(1), 0, 0, 0, cXYA(2);
	Dr = mmPerPulse*Er;
	Dl = mmPerPulse*El;

	if(n > 1){
		for(int i = 1; i < n; i++){
			dDr = Dr(i) - Dr(i-1);
			dDl = Dl(i) - Dl(i-1);
			dD = (dDr+dDl)/2;
			dA = (dDr+dDl)/wheelBaseMM;

			dX = dD*cos(posAdjXYA(i-1,2)+dA/2);
			dY = dD*sin(posAdjXYA(i-1,2)+dA/2);

			posAdjXYA(i,0) = posAdjXYA(i-1,0) + (dDr+dDl)/2*sin(posAdjXYA(i-1,2)+(dDr-dDl)/(2*wheelBaseMM)); //Swapped sin and cos
			posAdjXYA(i,1) = posAdjXYA(i-1,1) + (dDr+dDl)/2*cos(posAdjXYA(i-1,2)+(dDr-dDl)/(2*wheelBaseMM));
			posAdjXYA(i,2) = fmod(posAdjXYA(i-1,2) + (dDr-dDl)/wheelBaseMM, 2*M_PI);

			//Predict new uncertainty
			Sx.row(0) << cStorage(i-1,0), cStorage(i-1,1), cStorage(i-1,2);
			Sx.row(1) << cStorage(i-1,3), cStorage(i-1,4), cStorage(i-1,5);
			Sx.row(2) << cStorage(i-1,6), cStorage(i-1,7), cStorage(i-1,8);
			Sm << sigmaR*abs(dDr), 0,
				  0, sigmaL*abs(dDl);

			Js << 1, 0, -sin(posAdjXYA(i-1,2)-(dDl - dDr)/(2*wheelBaseMM))*(dDl/2 + dDr/2),
				  0, 1,  cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))*(dDl/2 + dDr/2),
				  0, 0,  1;

			Jm << cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))/2 - (sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))*(dDl/2 + dDr/2))/(2*wheelBaseMM), cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))/2 + (sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))*(dDl/2 + dDr/2))/(2*wheelBaseMM),
				  sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))/2 + (cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))*(dDl/2 + dDr/2))/(2*wheelBaseMM), sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))/2 - (cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))*(dDl/2 + dDr/2))/(2*wheelBaseMM),
				  1/wheelBaseMM,                                                                       															 -1/wheelBaseMM;

			Jb << -(sin(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))*(dDl - dDr)*(dDl/2 + dDr/2))/(2*pow(wheelBaseMM,2)),
				  (cos(posAdjXYA(i-1,2) - (dDl - dDr)/(2*wheelBaseMM))*(dDl - dDr)*(dDl/2 + dDr/2))/(2*pow(wheelBaseMM,2)),
				  (dDl - dDr)/pow(wheelBaseMM,2);

			cAdjXYA = Js*Sx*Js.transpose() + Jm*Sm*Jm.transpose() + Jb*Sb*Jb.transpose();
			cStorage.row(i) << cAdjXYA(0,0), cAdjXYA(0,1), cAdjXYA(0,2),
							   cAdjXYA(1,0), cAdjXYA(1,1), cAdjXYA(1,2),
							   cAdjXYA(2,0), cAdjXYA(2,1), cAdjXYA(2,2);
		}
		cXYA = cAdjXYA;
		posXYA = posAdjXYA.row(posAdjXYA.rows()-1);
		cout << posXYA << endl;
		cout << cXYA << endl;
	}
	pos_log << "###Movement values in mm###" << endl;
	pos_log << "X\tY\tA\t\tsX\tsY\tsA" << endl;
	for(int i = 0; i < n; i++){
	   pos_log << posAdjXYA(i,0) << "\t" << posAdjXYA(i,1) << "\t" << posAdjXYA(i,2) << "\t\t" << cStorage(i,0) << "\t" << cStorage(i,4) << "\t" << cStorage(i,8) << endl;
	}
	pos_log << "---END---" << endl;
	return 0;
}

int sendEncoderValues(VectorXd Er, VectorXd El){
   int fd, result;
   int Counter = 0;
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
	   Counter++;
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
	   usleep(10000); // sleep in micro sek
   }
//   cout << "Done!, endpos = (" << pWhite_Board_RX->Position_M0 << ",\t" << pWhite_Board_RX->Position_M1 << ")"<< endl;
   updatePos(Er, El);
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
		usleep(5000); // sleep in micro sek
	  }
	cout << "Done!\n";
}

int travel(VectorXd startXYA, VectorXd endXYA){


	VectorXd El1;	// Part1, 2 and 3 of encoder values
	VectorXd Er1;
	VectorXd El2;
	VectorXd Er2;
	VectorXd El3;
	VectorXd Er3;

	// Convert dX, dY and dA to dD and dA
	double dX = endXYA(0) - startXYA(0);
	double dY = endXYA(1) - startXYA(1);
	if(dY == 0){
		dY = 0.00000000001;
	}
	double dA = fmod(atan2(dX,dY)-startXYA(2),2*M_PI); //atan(y/x)
	double dD = sqrt(pow(dX,2)+pow(dY,2));

	// Rotate so there is a straight line from start to end
	if(dA > 0 && abs(dA)>M_PI/64){
		// Turn right
		cout << "turn right in the start\n";
		double dDl = abs(wheelBaseMM*dA);
		El1 = generateEncoderValues(dDl*turnAdjustmentFactor);
		Er1 = -El1;
		sendEncoderValues(Er1, El1);
	}else if(dA < 0 && abs(dA)>M_PI/64){
		// Turn left
		cout << "turn left in the start\n";
		double dDr = abs(wheelBaseMM*dA);
		Er1 = generateEncoderValues(dDr*turnAdjustmentFactor);
		El1 = -Er1;
		sendEncoderValues(Er1, El1);
	}
	// Travel distance dD
	cout << "Travel distance dD = " << dD << endl;
	cout << "gearReduction = " << gearReduction << endl;
	El2 = generateEncoderValues(dD);
	Er2 = El2;
	sendEncoderValues(Er2, El2);

	// Rotate to desired angle
	dA = endXYA(2) - dA;
	if(dA > 0){
		// Turn right
		cout << "turn right in the end\n";
		double dDl = abs(wheelBaseMM*dA);
		El3 = generateEncoderValues(dDl*turnAdjustmentFactor);
		Er3 = -El3;
		sendEncoderValues(Er3, El3);
	}else{
		// Turn left
		cout << "turn left in the end\n";
		double dDr = abs(wheelBaseMM*dA);
		Er3 = generateEncoderValues(dDr*turnAdjustmentFactor);
		El3 = -Er3;
		sendEncoderValues(Er3, El3);
	}
	return 0;
}

int main(){
	pos_log.open("logs/pos_controller/pos_log.txt", ofstream::out | ofstream::trunc);
	pWhite_Board_RX = (TYPE_White_Board_RX *)buffer;
	pWhite_Board_TX = (TYPE_White_Board_TX *)malloc(sizeof(TYPE_White_Board_TX));
	if(!pWhite_Board_TX){
		cout << "Couldn't allocate" << endl;
		exit(1);
	}
	VectorXd startXYA(3), endXYA(3);
	cXYA << 1, 0, 0,
			0, 1, 0,
			0, 0, pow(M_PI/180,2);
	posXYA << 1215, 160, 0;
	startXYA = posXYA;
	endXYA << 1215, 160+500, 0;
	resetEncoders();
	travel(startXYA, endXYA);
	travel(endXYA, startXYA);
//	resetEncoders();
	pos_log.close();
	return 0;
}



