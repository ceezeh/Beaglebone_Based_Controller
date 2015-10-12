#ifndef JOYSTICK_H_
#define JOYSTICK_H_
#include <unistd.h>
#include<iostream>
#include<fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"

using namespace std;
/***********************/
void* JSthreadedPoll(void *val);

/***********************/
class Joystick {
public:
	Joystick(int speedpin, int dirpin, const char * topic, ros::NodeHandle &n_t);
	int start(int rate);
	void stop();

private:
	float readAnalog(int pin);
	void pollOnce();
	int rate; // polling rate in Hz
	int speed_ain, dir_ain;
	int speed_offset, dir_offset;
	void getOffset();

	bool isrunning;
	pthread_t thread;
	friend void* JSthreadedPoll(void *val);
	pthread_attr_t tattr;


	ros::Publisher command_pub;
	ros::NodeHandle n;
	const char * topic;
	geometry_msgs::TwistStamped cmd;

};

#endif
