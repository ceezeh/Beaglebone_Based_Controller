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

int main(int argc, char **argv) {
	string ns = "controller_input";
	string topic = "input";
	ros::init(argc, argv, ns.c_str());
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	string interface = ns + "/interface";
	n.getParam(interface.c_str(), interface);
	cout << "Interface: :" << interface << endl;

	HeadArray headarray(48, 31, 30, topic.c_str(), n);
	Sippuff sippuff(50, 60, topic.c_str(), n);
	Joystick joystick(0, 2, topic.c_str(), n);

	if (interface == "HA") {
		headarray.start();
	} else if (interface == "SP") {
		sippuff.start();
	} else if (interface == "JS") {
		joystick.start(20);
	}

	while (ros::ok()) {
		loop_rate.sleep();
	}
}
