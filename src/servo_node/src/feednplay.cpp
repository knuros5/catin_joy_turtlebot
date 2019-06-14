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
#define SERVO_FEED 6
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
	//string mode = msg->data.c_str();
	ROS_INFO("bkr msg: %s\n", msg->data.c_str());

	if (!strcmp(msg->data.c_str(), "1")){	// mode 1: feeding
		feed();	
	}
	else if(!strcmp(msg->data.c_str(), "2")){	// mode 2: wiggling
		playWithFeather();
	}
}


void feed()
{
	ROS_INFO("please feed.\n");
	
}


void playWithFeather()
{
	ROS_INFO("## Playing with feather.\n");
	
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
