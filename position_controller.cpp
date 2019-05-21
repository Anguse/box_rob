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
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

// Robot specs
double wheelBaseMM = 170;
double uD = 52/340;  //uncertainty of 52 mm in 240 mm

// SPI
static const int CHANNEL = 1;
TYPE_White_Board_RX *pWhite_Board_RX;
TYPE_White_Board_TX *pWhite_Board_TX;
unsigned char buffer[100];

// M0 is the left motor
int sendEncoderValues(VectorXd Er, VectorXd El)
{
   int fd, result;
   int Counter = 0;
   FILE *fid, *pos_fid;
   int Done = 0;
   int toggle = 1;
   int startPosL, startPosR;
   int desPosL, desPosR;

   fid = fopen("temp","w");

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
	   fprintf(fid,"%d  %d %d\n",pWhite_Board_RX->Position_M0, desPosR, pWhite_Board_TX->Set_Speed_M0);
	   usleep(10000); // sleep in micro sek
   }


   cout << "Done!, endpos = (" << pWhite_Board_RX->Position_M0 << ",\t" << pWhite_Board_RX->Position_M1 << ")"<< endl;
   fclose(fid);
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

int getEncoderPosM0(){
	return pWhite_Board_RX->Position_M0;
}

int getEncoderPosM1(){
	return pWhite_Board_RX->Position_M1;
}

int travel(VectorXd startXYA, VectorXd endXYA){

	double turnAdjustmentFactor = 0.85;

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
		El1 = generateEncoderValues(dDl)*turnAdjustmentFactor;
		Er1 = -El1*turnAdjustmentFactor;
		sendEncoderValues(Er1, El1);
	}else if(dA < 0 && abs(dA)>M_PI/64){
		// Turn left
		cout << "turn left in the start\n";
		double dDr = abs(wheelBaseMM*dA);
		Er1 = generateEncoderValues(dDr)*turnAdjustmentFactor;
		El1 = -Er1*turnAdjustmentFactor;
		sendEncoderValues(Er1, El1);
	}
	// Travel distance dD
	cout << "Travel distance dD\n";
	El2 = generateEncoderValues(dD);
	Er2 = El2;
	sendEncoderValues(Er2, El2);

	// Rotate to desired angle
	dA = endXYA(2) - dA;
	if(dA > 0){
		// Turn right
		cout << "turn right in the end\n";
		double dDl = abs(wheelBaseMM*dA);
		El3 = generateEncoderValues(dDl)*turnAdjustmentFactor;
		Er3 = -El3*turnAdjustmentFactor;
		sendEncoderValues(Er3, El3);
	}else{
		// Turn left
		cout << "turn left in the end\n";
		double dDr = abs(wheelBaseMM*dA);
		Er3 = generateEncoderValues(dDr)*turnAdjustmentFactor;
		El3 = -Er3*turnAdjustmentFactor;
		sendEncoderValues(Er3, El3);
	}

	return 0;
}

int main(){
	pWhite_Board_RX = (TYPE_White_Board_RX *)buffer;
	pWhite_Board_TX = (TYPE_White_Board_TX *)malloc(sizeof(TYPE_White_Board_TX));
	if(!pWhite_Board_TX){
		cout << "Couldn't allocate" << endl ;
		exit(1);
	}
	VectorXd startXYA(3), endXYA(3);
	startXYA << 1215, 160, 0;
	endXYA << 1700, 500, 0;
	resetEncoders();
	travel(startXYA, endXYA);
	resetEncoders();
	return 0;
}



