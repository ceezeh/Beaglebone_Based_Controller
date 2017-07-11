#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#define INVALIDCMD -2
#define equals(x, y) (abs(x-y) < 0.001)
geometry_msgs::TwistStamped input;

void inputCallback(geometry_msgs::TwistStamped cmd) {
	input.header = cmd.header;
	if (!equals(cmd.twist.linear.x, INVALIDCMD)) {
		// update v
		input.twist.linear.x = cmd.twist.linear.x;
	}

	if (!equals(cmd.twist.angular.z, INVALIDCMD)) {
		input.twist.angular.z = cmd.twist.angular.z;
	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "Repeater");
	ros::NodeHandle n;

	ros::Publisher input_pub = n.advertise < geometry_msgs::TwistStamped
			> ("user_command", 1);
	ros::Subscriber input_sub = n.subscribe("input", 1, inputCallback);

	ros::Rate loop_rate(20);

	while (ros::ok()) {
		ros::spinOnce();
		input.header.stamp = ros::Time::now();
		input_pub.publish(input);
		loop_rate.sleep();
	}
}
