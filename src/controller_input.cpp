#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include <vector>

#include <pthread.h>
#include <sstream>
#include "controller_input/sip_puff.h"
#include "controller_input/headarray.h"
#include "controller_input/joystick.h"

using namespace std;

typedef enum {
	joystick, headarray, sippuff
} Interface;
Interface interface = sippuff;
// Headarray pins = 48,31,30

int main(int argc, char **argv) {
	ros::init(argc, argv, "controller_input");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
//	HeadArray headarray(48, 31, 30, "user_command", n);
//	headarray.start();
//	  Sippuff sippuff(50, 60, "user_command",n );
//	  sippuff.start();
	  Joystick joystick(0,2,"user_command",n);
	  joystick.start(20);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}
