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

// Threads
pthread_t match;

// Network
int sockfd;

// Map
//ArrayXXf map(4, 4);
MatrixXd map(4,4);

// Cache
//ArrayXXf last_scan(MIN_SCANS, 3);
MatrixXd last_scan(MIN_SCANS, 3);


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
	VectorXd result(3);
	MatrixXd rot(2,2);

	rot.col(0) << 0, -1;
	rot.col(1) << 1, 0;

	MatrixXd U(map.cols(), 2);
	VectorXd R(map.cols());

	// Get unit vectors of all lines
	for(int kk = 0; kk < map.cols(); kk++){

		MatrixXd L1(2,2);
		MatrixXd L2(2,2);
		MatrixXd V(2,2);
		MatrixXd Ui(2,2);
		VectorXd Uii(2);
		VectorXd z(2);
		double Ri;

		L1.col(0) << map.row(kk).col(2), map.row(kk).col(2);	// [X1, Y1; X2, Y2]
		L1.col(1) << map.row(kk).col(1), map.row(kk).col(3);
		L2.col(0) << map.row(kk).col(0), map.row(kk).col(0);	// [X1, X1; Y1, Y1]
		L2.col(1) << map.row(kk).col(1), map.row(kk).col(1);

		V = rot*(L1-L2);
		Ui = V/V.norm();
		Uii << Ui.row(1).col(0), Ui.row(1).col(1);
		z << L1.row(1).col(0), L1.row(1).col(1);
		Ri = Uii.dot(z);

		R(kk) = Ri;
		U.row(kk) = Uii;
	}

	bool finished = false;

	// Main loop
	while(!finished){
		int inliers = 0;

		for(int ii = 0; ii < MIN_SCANS; ii++){
			double angle, dist, x, y;

			angle = last_scan(ii, 1)*M_PI/180;
			dist = last_scan(ii, 2);
			x = cos(angle)*dist;
			y = sin(angle)*dist;
		}
	}

	VectorXd scan_x(MIN_SCANS);
	VectorXd scan_y(MIN_SCANS);

	//	std::cout << "x: "<< scan_x << "\n";
	//	std::cout << "y: "<< scan_y << "\n";


	last_scan.col(0)*=M_PI/180;

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
	line << 0, 0, 0, 310;	//x1, y1, x2, y2
	map.row(0) = line;
	line << 0, 310, 260, 310;
	map.row(1) = line;
	line << 260, 310, 260, 0;
	map.row(2) = line;
	line << 260, 0, 0, 0;
	map.row(3) = line;

	start_lidar();
	sockfd = start_lidar_server();
	last_scan = get_scan(sockfd, SCAN_TIMEOUT_MS, MIN_SCANS);

	pthread_create(&match, NULL, cox_linefit, (void*)NULL);
	usleep(3000000);
	pthread_join(match, NULL);

	stop_lidar();
	close(sockfd);
	return 0;
}


