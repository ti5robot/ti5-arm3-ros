 /*************************************************************************
 * Contact: abhinavjain241@gmail.com, abhinav.jain@heig-vd.ch
 * Date: 28/06/2016
 *
 * This file contains source code to the client node of the ROS package
 * comm_tcp developed at LaRA (Laboratory of Robotics and Automation)
 * as part of my project during an internship from May 2016 - July 2016.
 *
 * (C) All rights reserved. LaRA, HEIG-VD, 2016 (http://lara.populus.ch/)
 ***************************************************************************/
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <iostream>
#define MESSAGE_FREQ 100
#define MESSAGE_FREQ_END 60
#include <sstream>  
#include <string>
int sockfd, portno, n, choice = 1;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[5120];
bool echoMode = false;
void error(const char *msg) {
    perror(msg);
    exit(0);
}
double next_velocitie[6]={0}; 

double ori_velocitie[6]={0} ;
double cha_velocitie[6]={0} ;
double now_velocitie[6]={0} ;

double next_position[6]={0}; 

double ori_position[6]={0} ;
double cha_position[6]={0} ;
double now_position[6]={0} ;

class Listener {
private:
    char topic_message[2048] = { 0 };
public:
    //void callback(const sensor_msgs::JointState::ConstPtr &msg);
    void callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);
    char* getMessageValue();
};
void Listener::callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg) {
    ros::Rate loop_rate2(MESSAGE_FREQ);
    ros::Rate loop_rate3(MESSAGE_FREQ_END);
    //std::cout<<msg->trajectory<<std::endl;
    int n = msg->trajectory[0].joint_trajectory.points.size();
    char str[5120];
    char str123[5120];
    float ww;
    float vv;
    float zs;
    float zs1;
    float zss =1.0;
    std::string end_position ="";

    for (int i = 1; i < n-1; i++) {
        ori_velocitie[0]=msg->trajectory[0].joint_trajectory.points[i].velocities[0] ;
        ori_velocitie[1]=msg->trajectory[0].joint_trajectory.points[i].velocities[1] ;
        ori_velocitie[2]=msg->trajectory[0].joint_trajectory.points[i].velocities[2] ;
        ori_velocitie[3]=msg->trajectory[0].joint_trajectory.points[i].velocities[3] ;
        ori_velocitie[4]=msg->trajectory[0].joint_trajectory.points[i].velocities[4] ;
        ori_velocitie[5]=msg->trajectory[0].joint_trajectory.points[i].velocities[5] ;
        next_velocitie[0]=msg->trajectory[0].joint_trajectory.points[i+1].velocities[0] ;
        next_velocitie[1]=msg->trajectory[0].joint_trajectory.points[i+1].velocities[1] ;
        next_velocitie[2]=msg->trajectory[0].joint_trajectory.points[i+1].velocities[2] ;
        next_velocitie[3]=msg->trajectory[0].joint_trajectory.points[i+1].velocities[3] ;
        next_velocitie[4]=msg->trajectory[0].joint_trajectory.points[i+1].velocities[4] ;
        next_velocitie[5]=msg->trajectory[0].joint_trajectory.points[i+1].velocities[5] ;
        cha_velocitie[0]=ori_velocitie[0]-next_velocitie[0];
        cha_velocitie[1]=ori_velocitie[1]-next_velocitie[1];
        cha_velocitie[2]=ori_velocitie[2]-next_velocitie[2];
        cha_velocitie[3]=ori_velocitie[3]-next_velocitie[3];
        cha_velocitie[4]=ori_velocitie[4]-next_velocitie[4];
        cha_velocitie[5]=ori_velocitie[5]-next_velocitie[5];
         
        ori_position[0]=msg->trajectory[0].joint_trajectory.points[i].positions[0] ;
        ori_position[1]=msg->trajectory[0].joint_trajectory.points[i].positions[1] ;
        ori_position[2]=msg->trajectory[0].joint_trajectory.points[i].positions[2] ;
        ori_position[3]=msg->trajectory[0].joint_trajectory.points[i].positions[3] ;
        ori_position[4]=msg->trajectory[0].joint_trajectory.points[i].positions[4] ;
        ori_position[5]=msg->trajectory[0].joint_trajectory.points[i].positions[5] ;
        next_position[0]=msg->trajectory[0].joint_trajectory.points[i+1].positions[0] ;
        next_position[1]=msg->trajectory[0].joint_trajectory.points[i+1].positions[1] ;
        next_position[2]=msg->trajectory[0].joint_trajectory.points[i+1].positions[2] ;
        next_position[3]=msg->trajectory[0].joint_trajectory.points[i+1].positions[3] ;
        next_position[4]=msg->trajectory[0].joint_trajectory.points[i+1].positions[4] ;
        next_position[5]=msg->trajectory[0].joint_trajectory.points[i+1].positions[5] ;
        cha_position[0]=ori_position[0]-next_position[0];
        cha_position[1]=ori_position[1]-next_position[1];
        cha_position[2]=ori_position[2]-next_position[2];
        cha_position[3]=ori_position[3]-next_position[3];
        cha_position[4]=ori_position[4]-next_position[4];
        cha_position[5]=ori_position[5]-next_position[5];
        std::cout << ros::Time::now()<<std::endl;
        std::string totle_per_position ="";
        for (int ii=0;ii<1;ii++){
             vv=fabs(ori_velocitie[0]);
             ww=fabs(cha_position[0])*180/9.1415926;
             if (vv>0.01){
             zs=vv;
             zs1=zs*3.1415926;
             zss=1.0;//ww betrwing to point ----    vv ros caculate The speed ----zss distance/** =time
             std::cout <<"cha_position:" <<ww<<std::endl;
             std::cout <<"angel speed:" <<zs1<<std::endl;
              ros::Duration(round(1000)/6000.0).sleep();
              }

            sprintf(str,"%.1lf&%.1lf %.1lf&%.1lf %.1lf&%.1lf %.1lf&%.1lf %.1lf&%.1lf %.1lf&%.1lf,",ori_position[0]/6.28*101*65536,ori_velocitie[0]*1250,ori_position[1]/6.28*101*65536,ori_velocitie[1]*1250,ori_position[2]/6.28*101*65536,ori_velocitie[2]*1250,ori_position[3]/6.28*101*65536,ori_velocitie[3]*1250,ori_position[4]/6.28*101*65536,ori_velocitie[4]*1250,ori_position[5]/6.28*101*65536,ori_velocitie[5]*1250);

            totle_per_position = totle_per_position +str;
            std::cout << str<<std::endl;
            send(sockfd,totle_per_position.c_str(),strlen(totle_per_position.c_str()),0);
             
        }
            totle_per_position="";
            
            if (strcmp(buffer, "exit") == 0) {
                std::cout << "...disconnect" << std::endl;
                break;
            }
        
}
}

char* Listener::getMessageValue() {
    return topic_message;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "client_node");
	ros::NodeHandle nh;
    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ
	Listener listener;
        ros::Subscriber client_sub = nh.subscribe("/move_group/display_planned_path", 1000, &Listener::callback, &listener);
    if (argc < 3) {
       fprintf(stderr,"Usage: $ rosrun comm_tcp client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
       exit(0);
    }
    if (argc > 3)
		if (strcmp(argv[3], "-e") == 0)
			echoMode = true;
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    std::cout << "How do you want the client to behave?:\n1. Be able to send messages manually\n2. Subscribe to /client_messages and send whatever's available there\nYour choice:";
    int choice=2;
	while(ros::ok()) {
        if (choice == 1) {
            printf("Please enter the message: ");
            fgets(buffer,2048,stdin);
        } else if (choice == 2) {
            strcpy(buffer, listener.getMessageValue());
            loop_rate.sleep();
        }
	    ros::spinOnce();
	}
	return 0;
}
