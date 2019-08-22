/*
 * main.cpp
 *
 *  Created on: 30 apr. 2019
 *      Author: harald
 */
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <cstdlib>
#include "lidar.h"
#include "constants.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Threads
pthread_t match;
pthread_mutex_t coxlock;

// Network
int sockfd;

// Map
MatrixXd line_map(4,4);

// Cache
MatrixXd last_scan(MIN_SCANS, 3);
VectorXd cox_adjustment(3);			// Adjustment calculated by cox
VectorXd coxPosXYA(3);				// Current position of robot used when calculating cox
MatrixXd coxVariance(3,3);
bool cox_done;



// Logging
ifstream inputFile;
ofstream adjustments, measurements, positions, prerot_measurements, scan_inliers, raw, lidar_variance, failed_cox;

int start_lidar(){
	struct sockaddr_in serv_addr;
	int sock;
	unsigned char command[2] = {'\x10','\x00'};

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("Socket creation failed");
		exit(EXIT_FAILURE);
	}

	memset(&serv_addr, '0', sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(SEND_PORT);

	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, HOST, &serv_addr.sin_addr)<=0)
	{
		perror("Adress not supported");
		exit(EXIT_FAILURE);
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Connection failed");
		exit(EXIT_FAILURE);
	}

	if (send(sock, reinterpret_cast<const char*>(command), sizeof(command), 0) < 0){
		perror("Send failed");
		exit(EXIT_FAILURE);
	}
	close(sock);
	return EXIT_SUCCESS;
}

int stop_lidar(){
	struct sockaddr_in serv_addr;
	int sock;
	unsigned char command[2] = {'\x20','\x00'};

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("Socket creation failed");
		exit(EXIT_FAILURE);
	}

	memset(&serv_addr, '0', sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(SEND_PORT);

	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, HOST, &serv_addr.sin_addr)<=0)
	{
		perror("Adress not supported");
		exit(EXIT_FAILURE);
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Connection failed");
		exit(EXIT_FAILURE);
	}

	if (send(sock, reinterpret_cast<const char*>(command), sizeof(command), 0) < 0){
		perror("Send failed");
		exit(EXIT_FAILURE);
	}
	close(sock);
	return EXIT_SUCCESS;
}

int start_lidar_server(){
	struct sockaddr_in serv_addr, cli_addr;
	socklen_t clilen;
	int sockfd, newsockfd;

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(RECEIVE_PORT);

	if (bind(sockfd, (struct sockaddr *)&serv_addr,
			sizeof(serv_addr))<0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	printf("bind success\n");
	if (listen(sockfd, 5) < 0)
	{
		perror("listen failed");
		exit(EXIT_FAILURE);
	}
	printf("listen success\n");
	clilen = sizeof(cli_addr);
	if ((newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen))<0)
	{
		perror("rejected");
		exit(EXIT_FAILURE);
	}
	printf("accepted\n");
	close(sockfd);
	return newsockfd;
}

void interrupt_handler(int s){
	inputFile.close();
	adjustments.close();
	measurements.close();
	prerot_measurements.close();
	positions.close();
	raw.close();
	stop_lidar();
	pthread_mutex_destroy(&coxlock);
	close(sockfd);
	exit(1);
}

MatrixXd get_scan(int sockfd, int scan_duration_ms, int min_scans){

	MatrixXd all_scans(min_scans, 3);
	VectorXd one_scan(3);
	struct timeval tp;
	char header[5];
	char data[4096];
	int angle, quality, distance, number_of_ch, number_of_data;
	int number_of_scans = 0;

	gettimeofday(&tp, NULL);
	long int start_time_ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	long int current_time_ms = start_time_ms;
	while(current_time_ms < (start_time_ms + scan_duration_ms) && number_of_scans < min_scans){
		bzero(header, 5);
		number_of_ch = read(sockfd, header, 5);
		if(number_of_ch != 5){
			break;
		}
		if((int)header[0] == 165){
			number_of_data = (((int)header[2]<<16) + ((int)header[3]<<8) + ((int)header[4]));
			read(sockfd, data, number_of_data);
			quality = (int)data[0]>>2;
			angle = (((int)data[1]>>1) + ((int)data[2]<<8))>>7;
			distance = (((int)data[3]) + ((int)data[4]<<8))>>2;
			one_scan << quality, M_PI+fmod(angle*SCAN_ANGLE_ADJUSTMENT+M_PI,M_PI*2), distance;
			if(quality != 1){
				all_scans.row(number_of_scans) = one_scan;
				raw << one_scan(0) << "\t" << one_scan(1) << "\t" << one_scan(2) << "\n";
				number_of_scans++;
			}
		}
		gettimeofday(&tp, NULL);
		current_time_ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	}
	return all_scans;
}

void cox_thread_start(void *arg){


	return;
}

void *cox_linefit(void *ptr){

	int iterations = 0;
	int nr_of_lines = line_map.cols();

	double ddx = 0;					// Adjustment to initial position
	double ddy = 0;
	double dda = 0;

	int inlier_threshold_offset = 0;

	MatrixXd U(nr_of_lines, 2);		// Unit vectors
	VectorXd R(nr_of_lines);		// Unit vectors projected on corresponding line
	MatrixXd rot(2,2);				// 90 Degree rotation matrix
	MatrixXd rob_rot(3,3);			// Rotation matrix, sensor coordinates -> robot coordinates
	MatrixXd world_rot(3,3);		// Rotation matrix, robot coordinates -> world coordinates
	VectorXd v(2);					// Point [X, Y]
	MatrixXd vi(1,3);				// Point and target index if inlier [X, Y, idx]
	VectorXd state(3);				// The adjusted position of the robot
	VectorXd Xw(3); 				// World coordinates
	MatrixXd Xww(MIN_SCANS,3);		// World coordinates, All
	VectorXd vm(2);					// Median of all points
	VectorXd yy(nr_of_lines);		// Distance from point to lines
	VectorXd targets(1);			// All targets
	VectorXd X1(1), X2(1), X3(1);	// Input to solve least square fit
	MatrixXd midpoint(nr_of_lines,2);
	VectorXd linelength(nr_of_lines);

	double angle, dist, x, y;

	state << coxPosXYA(0), coxPosXYA(1), coxPosXYA(2);		// Initial position

	rot << 0, -1,
		   1, 0;
	rob_rot << cos(GAMMA), -sin(GAMMA), ALPHA,
			   sin(GAMMA), cos(GAMMA), 	BETA,
			   0,		   0, 		    1;

	// Get unit vectors of all lines
	for(int kk = 0; kk < nr_of_lines; kk++){

		MatrixXd L1(2,2);
		MatrixXd L2(2,2);
		MatrixXd V(2,2);
		MatrixXd Ui(2,2);
		VectorXd Uii(2);
		VectorXd z(2);
		double Ri;

		L1 << line_map(kk,0), line_map(kk,1),	//[X1, Y1,
			  line_map(kk,2), line_map(kk,3); 	// X2, Y2]
		L2 << line_map(kk,0), line_map(kk,1), 	//[X1, Y1,
			  line_map(kk,0), line_map(kk,1); 	// X1, Y1]

		V = rot*(L1-L2);
		Ui = V/V.norm();
		Uii << Ui(2), Ui(0); //0,2
		z << L1(1,0), L1(1,1);
		Ri = z.dot(Uii);

		R(kk) = Ri;
		U.row(kk) = Uii;

		midpoint.row(kk) << (L1(0,0)+L1(0,1))/2, (L1(1,0)+L1(1,1))/2;
		linelength(kk) = (L1.row(0)+L1.row(1)).norm();
	}

	for(int ii = 0; ii < MIN_SCANS; ii++){
		angle = last_scan(ii, 1);
		dist = last_scan(ii, 2);
		x = cos(angle)*dist;
		y = sin(angle)*dist;
		Xw << x, y, 1;
		Xw = rob_rot*Xw;
		Xw = world_rot*Xw;
		v << Xw(0), Xw(1);
		measurements << v(0) << "\t" << v(1) << "\n";
	}

	// Main loop
	bool finished = false;
	while(!finished){

	    double dx, dy, da;
		double target;
	    int n;
		int inliers = 0;
		int target_index;

	    VectorXd B(3);
	    MatrixXd S2(3,3);
	    MatrixXd C;

		world_rot << cos(state(2)), -sin(state(2)), state(0),
					 sin(state(2)), cos(state(2)),  state(1),
					 0, 			   0, 				  1;

		for(int ii = 0; ii < MIN_SCANS; ii++){
			angle = last_scan(ii, 1);
			dist = last_scan(ii, 2);
			x = cos(angle)*dist;
			y = sin(angle)*dist;
			Xw << x, y, 1;
			Xw = rob_rot*Xw;
			Xw = world_rot*Xw;
			Xww.row(ii) = Xw;
			v << Xw(0), Xw(1);
		}
		vm << Xww.col(0).array().mean(), Xww.col(1).array().mean();

		for(int ii = 0; ii < MIN_SCANS; ii++){
			angle = last_scan(ii, 1);
			dist = last_scan(ii, 2);
			x = cos(angle)*dist;
			y = sin(angle)*dist;
			prerot_measurements << x << "\t" << y << "\n";
			Xw << x, y, 1;
			Xw = rob_rot*Xw;
			Xw = world_rot*Xw;
			v << Xw(0), Xw(1);
			for(int kk = 0; kk < nr_of_lines; kk++){
				yy(kk) = abs(R(kk)-(U.row(kk).dot(v)));
			}
			target = yy.minCoeff(&target_index);
			if(ii == 9){
//				cout << "index: " << target_index << ", distance: " << target << "\n";
//				target_index = fmod(target_index+1, 4);
//				cout << "unit vector: " << U.row(target_index) << "\n";
			}
			if((target<INLIER_THRESHOLD+inlier_threshold_offset)/*&&sqrt(pow((Xw(0)-midpoint(target_index,0)),2))+sqrt(pow((Xw(1)-midpoint(target_index,1)),2))<=linelength(target_index)*/){
				inliers++;
				X1.conservativeResize(inliers);		//Time consuming...
				X2.conservativeResize(inliers);
				X3.conservativeResize(inliers);
				vi.conservativeResize(inliers, 3);
				targets.conservativeResize(inliers);
				X1(inliers-1) = U(target_index,0);
	            X2(inliers-1) = U(target_index,1);
	            X3(inliers-1) = U.row(target_index)*rot*(v-vm);
	            targets(inliers-1) = target;
	            vi.row(inliers-1).col(0) << v(0);
				vi.row(inliers-1).col(1) << v(1);
	            vi(inliers-1, 2) = target_index;
	            scan_inliers << v.transpose() << "\n";
			}
		}
		if(iterations > MAX_ITERATIONS){
			finished = true;
			cox_adjustment(0) = 0;
			cox_adjustment(1) = 0;
			cox_adjustment(2) = 0;
			cox_done = true;
			std::cout << "Failed to find convergence\n";
			break;
		}
		if(inliers < MIN_INLIERS){
			iterations++;
			inlier_threshold_offset+=2;
			cout << "not enough inliers, increasing threshold\n";
			continue;
		}
		if(inliers > MAX_INLIERS){
			iterations++;
			inlier_threshold_offset-=2;
			cout << "too many inliers, decreasing threshold\n";
			continue;
		}
		MatrixXd A(inliers, 3);
		A.col(0) = X1;
		A.col(1) = X2;
		A.col(2) = X3;
		B = A.householderQr().solve(targets);
//		B = (A.transpose() * A).ldlt().solve(A.transpose() * targets);
		n = A.size();
		S2 = ((targets-(A*B)).transpose()*(targets-(A*B)))/((double)(n-4.0)); // Calculate the variance
		C = S2(0)*((A.transpose()*A).completeOrthogonalDecomposition().pseudoInverse());

		// Add latest contribution to the overall congruence
		dx = B(0);
		dy = B(1);
		da = B(2);

		ddx = ddx + dx;
		ddy = ddy + dy;
		dda = fmod(dda+da,2*M_PI);

		// Update the position
		state(0) += dx;
		state(1) += dy;
		state(2) = fmod(state(2)+da,2*M_PI);

		// If adjustment is too big we are on the wrong track, discard these adjustments
		if(abs(ddx)>DISPLACEMENT_LIMIT || (abs(ddy)>DISPLACEMENT_LIMIT) || (abs(dda) >ANGLE_CHANGE_LIMIT)){
			ddx = 0;
			ddy = 0;
			dda = 0;
			cox_adjustment(0) = ddx;
			cox_adjustment(1) = ddy;
			cox_adjustment(2) = dda;
			cox_done = true;
			finished = true;
			for(int i = 0; i < inliers; i++){
				failed_cox << vi(i,0) << "\t" << vi(i,1) << "\n";
			}
			coxVariance << 1, 0, 0,
						   0, 1, 0,
						   0, 0, 1*M_PI/180;
			std::cout << "Too big adjustment, " << inliers << " inliers\n";
			break;
		}
		// Check if process has converged
		if((sqrt(pow(dx,2)+pow(dy,2)) < CONVERGENCE_THRESHOLD)&&(abs(da<CONVERGENCE_ANGULAR_THRESHOLD))){
			finished = true;
			cox_adjustment(0) = ddx;
			cox_adjustment(1) = ddy;
			cox_adjustment(2) = dda;
			coxVariance = C;
			cox_done = true;
			cout << "Finished in " << iterations << "iterations with " << inliers << "inliers\n";
			break;
		}
		targets.setZero(1);
		X1.setZero(1);
		X2.setZero(1);
		X3.setZero(1);
		iterations++;
	}
	return NULL;
}

int lidarInit(const char* filepath){

	pthread_mutex_init(&coxlock, NULL);

	coxVariance << 10, 0, 0,
				   0, 10, 0,
				   0, 0, 5*M_PI/180;

	// Construct map
	VectorXd line(4);
	line << 0, 0, 0, 3635;			//x1, y1, x2, y2
	line_map.row(0) = line;
	line << 0, 3635, 2430, 3635;
	line_map.row(1) = line;
	line << 2430, 3635, 2430, 0;
	line_map.row(2) = line;
	line << 2430, 0, 0, 0;
	line_map.row(3) = line;

	adjustments.open("logs/lidar_adjustment.txt", ofstream::out | ofstream::trunc);
	measurements.open("logs/lidar_measurements.txt", ofstream::out | ofstream::trunc);
	prerot_measurements.open("logs/prerot_measurements.txt", ofstream::out | ofstream::trunc);
	positions.open("logs/lidar_positions.txt", ofstream::out | ofstream::trunc);
	scan_inliers.open("logs/lidar_inliers.txt", ofstream::out | ofstream::trunc);
	raw.open("logs/lidar_raw.txt", ofstream::out | ofstream::trunc);
	lidar_variance.open("logs/lidar_variance.txt", ofstream::out | ofstream::trunc);
	failed_cox.open("logs/lidar_failed.txt", ofstream::out | ofstream::trunc);

	if(!filepath){
		start_lidar();
		sockfd = start_lidar_server();
	}
	inputFile.open(filepath);

	cox_done = false;
	return 0;
}

int lidarStop(){
	pthread_mutex_destroy(&coxlock);
	inputFile.close();
	adjustments.close();
	measurements.close();
	prerot_measurements.close();
	positions.close();
	lidar_variance.close();
	failed_cox.close();
	stop_lidar();
	close(sockfd);
	return 0;
}

int lidarCoxStart(VectorXd currentPosXYA, bool RFF){

	coxPosXYA = currentPosXYA;
	cox_done = false;
	if(!RFF){
		last_scan = get_scan(sockfd, 500, MIN_SCANS);
	}else{
		// Read from file
		string readval;
		string::size_type sz;
		for(int i = 0; i < MIN_SCANS; i++){
//			getline(inputFile, readval, ' ');
//			getline(inputFile, readval, ' ');			// Quality
//			last_scan(i,0) = stod(readval, &sz);
//			getline(inputFile, readval, ' ');			// Angles
//			last_scan(i,1) = stod(readval, &sz);
//			last_scan(i,1) = last_scan(i,1)*SCAN_ANGLE_ADJUSTMENT;	// Degrees to radians
//			last_scan(i,1) = fmod(last_scan(i,1),2*M_PI);
//			getline(inputFile, readval, ' ');			// Distance
//			last_scan(i,2) = stod(readval, &sz);
		}
	}
//	for(int i = 0; i < last_scan.rows(); i++){
//		measurements << last_scan(i,0) << "\t" << last_scan(i,1) << "\t" << last_scan(i,2) << endl;
//	}
	pthread_create(&match, NULL, cox_linefit, (void*)NULL);
	return 0;
}

// Check if cox is done, in this case return a reference to this
bool lidarCoxDone(){
	return cox_done;
}

VectorXd lidarGetCoxAdj(){
	if(cox_done){
		adjustments << cox_adjustment(0) << "\t" << cox_adjustment(1) << "\t" << cox_adjustment(2) << endl;
		positions << coxPosXYA(0)+cox_adjustment(0) << "\t" << coxPosXYA(1)+cox_adjustment(1) << "\t" << coxPosXYA(2)+cox_adjustment(2) << endl;
		cox_done = false;
		return cox_adjustment;
	}else{
		VectorXd n(3);
		return n;
	}
}

MatrixXd lidarGetVariance(){
	lidar_variance << coxVariance(0,0) << "\t" << coxVariance(0,1) << "\t" << coxVariance(0,2) << "\t" << coxVariance(1,0) << "\t" << coxVariance(1,1) << "\t" << coxVariance(1,2) << "\t" << coxVariance(2,0) << "\t" << coxVariance(2,1) << "\t" << coxVariance(2,2) << "\n";
	return coxVariance;
}

void lidarSetVariance(MatrixXd newVariance){
	coxVariance = newVariance;
}
