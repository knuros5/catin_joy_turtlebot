/*
* Title: feednwiggle.cpp
* Contents: Subscribe flag to move servo motors.
* Author: Minjeong Kang
* Date: 2019/06/14
*/

/* Header for ros */
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sstream>
#include<string.h>
#include<string>

/* Header for GPIO */
#include<stdio.h>
#include<wiringPi.h>
#include<softPwm.h>

/* Macros */
#define DEGREE(n) 15 + (n / 10)
#define INTERVAL 400000
#define SERVO_FEED 4
#define SERVO_PLAY 5

void servoCallback(const std_msgs::String::ConstPtr& msg);
void feed();
void playWithFeather();

int main(int argc, char **argv)
{
	/* ros initialization */
	ros::init(argc, argv, "feednplay");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("servo_motion", 1000, servoCallback);
	
	/* wiringPi initialization */
	setenv("WIRINGPI_GPIOMEM", "1", 1);     // sudo previlege
	if(wiringPiSetup() == -1){
                ROS_INFO("Error: something wrong with wiringPi.\n");
                return 1;
        }
	
	ros::spin();

	return 0;
}


void servoCallback(const std_msgs::String::ConstPtr& msg)
{
	if (!strcmp(msg->data.c_str(), "1")){	// mode 1: feeding
		printf("[feednplay/servo] FEEDING.\n");
		feed();	
	}
	else if(!strcmp(msg->data.c_str(), "2")){	// mode 2: playing
		printf("[feednplay/servo] PLAYING WITH FEATHER.\n");
		playWithFeather();
	}
}


void feed()
{
	softPwmCreate(SERVO_FEED, 0, 200);

        while(1){
                softPwmWrite(SERVO_FEED, DEGREE(90));
                usleep(INTERVAL);
                softPwmWrite(SERVO_FEED, DEGREE(-90));
                usleep(INTERVAL*2);
        }
}


void playWithFeather()
{
	softPwmCreate(SERVO_PLAY, 0, 200);	
	softPwmWrite(SERVO_PLAY, DEGREE(0));

	for(int i = 0; i < 3; i++){
		softPwmWrite(SERVO_PLAY, DEGREE(30));
		usleep(INTERVAL);
		softPwmWrite(SERVO_PLAY, DEGREE(-30));
		usleep(INTERVAL);
		softPwmWrite(SERVO_PLAY, DEGREE(15));
                usleep(INTERVAL);
		softPwmWrite(SERVO_PLAY, DEGREE(-15));
		usleep(INTERVAL);
	}
}
