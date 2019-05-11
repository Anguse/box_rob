/*
 * main.cpp
 *
 *  Created on: 30 apr. 2019
 *      Author: harald
 */
#include <iostream>
#include <eigen3/Eigen/Dense>
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

#define RECEIVE_PORT 9888
#define SEND_PORT 9887
#define HOST "127.0.0.1"

#define MIN_SCANS 100
#define SCAN_TIMEOUT_MS 10000

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
VectorXd cox_adjustment(3);
bool cox_done;


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
	stop_lidar();
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
			angle = ((int)data[1]>>1) + ((int)data[2]>>8);
			distance = ((int)data[3]) + ((int)data[4]>>8);
			one_scan << quality, angle, distance;
			if(quality != 1){
				all_scans.row(number_of_scans) = one_scan;
				number_of_scans++;
			}
		}
		gettimeofday(&tp, NULL);
		current_time_ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	}
	return all_scans;
}

void *cox_linefit(void *ptr){

	int iterations = 0;
	int nr_of_lines = line_map.cols();
	double threshold = 10;			// Fixed threshold value
	double ddx = 0;					// Adjustment to initial position
	double ddy = 0;
	double dda = 0;

	int alfa = 0;					// Robot offset
	int gamma = 0;
	int beta = (-90*M_PI)/180;

	MatrixXd U(nr_of_lines, 2);		// Unit vectors
	VectorXd R(nr_of_lines);		// Unit vectors projected on corresponding line
	MatrixXd rot(2,2);				// 90 Degree rotation matrix
	MatrixXd rob_rot(3,3);			// Rotation matrix, sensor coordinates -> robot coordinates
	MatrixXd world_rot(3,3);		// Rotation matrix, robot coordinates -> world coordinates
	VectorXd v(2);					// Point [X, Y]
	MatrixXd vi(1,3);				// Point and target index if inlier [X, Y, idx]
	VectorXd state(3);				// The adjusted position of the robot
	VectorXd Xw(3); 				// World coordinates
	VectorXd vm(2);					// Median of all points
	VectorXd yy(nr_of_lines);		// Distance from point to lines
	VectorXd targets(1);			// All targets
	VectorXd X1(1), X2(1), X3(1);	// Input to solve least square fit

	state << 50, 140, 90*M_PI/180; 	// Initial position
	vm << state(1), state(2);

	rot << 0, -1,
		   1, 0;
	rob_rot << cos(gamma), -sin(gamma), alfa,
			   sin(gamma), cos(gamma), 	beta,
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
		Uii << Ui(0), Ui(2);
		z << L1(1,0), L1(1,1);
		Ri = Uii.dot(z);

		R(kk) = Ri;
		U.row(kk) = Uii;
	}

	// Main loop
	bool finished = false;
	while(!finished){

		double angle, dist, x, y;
	    double dx, dy, da;
		double target;
	    int n;
		int inliers = 0;
		int target_index;

	    VectorXd B(3);
	    MatrixXd S2(3,3);

		world_rot << cos(state(2)), -sin(state(2)), state(0),
					 sin(state(2)), cos(state(2)),  state(1),
					 0, 			   0, 				  1;

		for(int ii = 0; ii < MIN_SCANS; ii++){
			angle = last_scan(ii, 1)*M_PI/180;
			dist = last_scan(ii, 2);
			x = cos(angle)*dist;
			y = sin(angle)*dist;
			Xw << x, y, 1;
			Xw = rob_rot*Xw;
			Xw = world_rot*Xw;
			v << Xw(0), Xw(1);
			for(int kk = 0; kk < nr_of_lines; kk++){
				yy(kk) = abs(R(kk)-(U.row(kk).dot(v)));
			}
			target = yy.minCoeff(&target_index);
			if(target<threshold){
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
			}
		}
		if(inliers > 0){
			MatrixXd A(inliers, 3);
			A.col(0) = X1;
			A.col(1) = X2;
			A.col(2) = X3;
			B = (A.transpose() * A).ldlt().solve(A.transpose() * targets);
			n = A.size();
	//	    S2 = ((targets-A*B).transpose()*(targets-A*B))/(n-4); // Calculate the variance
	//	    C = S2*(inv(A'*A));

			// Add latest contribution to the overall congruence
			dx = B(0);
			dy = B(1);
			da = B(2);

			ddx = ddx + dx;
			ddy = ddy + dy;
			dda = dda + da;

		    // Update the position
			state(0) += dx;
			state(1) += dy;
			state(2) += da;

		}
		    // Check if process has converged
			if((sqrt(pow(dx,2)+pow(dy,2)) < 5)&&(abs(da<0.1*M_PI/180))){
				finished = true;
				cox_adjustment(0) = ddx;
				cox_adjustment(1) = ddy;
				cox_adjustment(2) = dda;
				pthread_mutex_lock(&coxlock);
				cox_done = true;
				pthread_mutex_unlock(&coxlock);
				std::cout << "Finished in " << iterations << "iterations\n";
				break;
			}
			if(iterations > 200){
				finished = true;
				cox_adjustment(0) = 0;
				cox_adjustment(1) = 0;
				cox_adjustment(2) = 0;
				pthread_mutex_lock(&coxlock);
				cox_done = true;
				pthread_mutex_unlock(&coxlock);
				std::cout << "Failed to find convergence\n";
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

int main(){

	// Interrupt handling to make sure lidar stops on SIGINT
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = interrupt_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// Construct map
	VectorXd line(4);
	line << 0, 0, 0, 580;	//x1, y1, x2, y2
	line_map.row(0) = line;
	line << 0, 580, 380, 580;
	line_map.row(1) = line;
	line << 380, 580, 380, 0;
	line_map.row(2) = line;
	line << 380, 0, 0, 0;
	line_map.row(3) = line;

	start_lidar();
	sockfd = start_lidar_server();
//	usleep(2000000);
//	last_scan = get_scan(sockfd, match, MIN_SCANS);
//	stop_lidar();
//	close(sockfd);
//	cox_done = false;
	pthread_mutex_init(&coxlock, NULL);
	while(1){
		cox_done = false;
		last_scan = get_scan(sockfd, match, MIN_SCANS);
		pthread_create(&match, NULL, cox_linefit, (void*)NULL);
		while(1){
//			pthread_mutex_lock(&coxlock);
			if(cox_done){
				break;
			}
//			pthread_mutex_unlock(&coxlock);
		}
		cout << cox_adjustment << "\n";
	}
//	pthread_create(&match, NULL, cox_linefit, (void*)NULL);
//	usleep(3000000);
//	pthread_join(match, NULL);
//	cout << cox_adjustment << "\n";
	stop_lidar();
	close(sockfd);
	exit(1);
}


