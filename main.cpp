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
#include <string.h>

#define RECEIVE_PORT 9888
#define SEND_PORT 9887
#define HOST "127.0.0.1"

// Threads
pthread_t match;

//void *cox_linefit(void *param)(
//
//
//	//Make laser scan
//
//	//Calc linefit
//
//	//Update global variables and flag
//
//);

using Eigen::MatrixXd;

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

int laser_scan(int scan_duration_ms){
	struct sockaddr_in serv_addr, cli_addr;
	socklen_t clilen;
	struct timeval tp;
	int valread, sockfd, newsockfd, number_of_ch, number_of_data;
	char buffer[1024] = {0};
	char header[5];
	char data[4096];
	int angle, quality, distance;

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket failed");
		return -1;
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
	gettimeofday(&tp, NULL);
	long int start_time_ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	long int current_time_ms = start_time_ms;
	while(current_time_ms < (start_time_ms + scan_duration_ms)){
		bzero(header, 5);
		number_of_ch = read(newsockfd, header, 5);
		if(number_of_ch != 5){
			break;
		}
		if((int)header[0] == 165){
			number_of_data = (((int)header[2]<<16) + ((int)header[3]<<8) + ((int)header[4]));
			read(newsockfd, data, number_of_data);
			quality = (int)data[0]>>2;
			angle = ((int)data[1]>>1) + ((int)data[2]>>8);
			distance = ((int)data[3]) + ((int)data[4]>>8);
			printf("%d, %d, %d\n", quality, angle, distance);
		}
		gettimeofday(&tp, NULL);
		current_time_ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	}
    close(newsockfd);
	close(sockfd);
	return EXIT_SUCCESS;
}

int main(){

	int scan_duration = 10000;

	start_lidar();
	laser_scan(scan_duration);
	stop_lidar();

	Eigen::ArrayXXf map(4, 4);
	Eigen::ArrayXf v(4);
	v << 0, 0, 0, 310;
	map.row(0) = v;
	v << 0,310,260,310;
	map.row(1) = v;
	v << 260,310,260,0;
	map.row(2) = v;
	v << 260,0,0,0;
	map.row(3) = v;
	std::cout << map;

	return 0;


}


