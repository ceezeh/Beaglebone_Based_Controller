#include "controller_input/headarray.h"
#define THRESHOLD 300 //ms
#define INVALIDCMD -2
#define SPEEDINCR 0.2
#define DIRINCR 0.05
#define equals(x, y) (abs(x-y) < 0.001)
#define LINSPEED 0.7
#define ANGSPEED 0.5
#define TURNSPEED 0.05
using namespace std;

using namespace exploringBB;

HeadArray::HeadArray(int left_pin, int centre_pin, int right_pin,
		const char * topic, ros::NodeHandle &n_t) {
	n = n_t;
	this->topic = topic;



	statetimer = new StateTimer();

	threshold = THRESHOLD;

	centreBtn = new Button(centre_pin,
			boost::bind(&HeadArray::onCQuickpress, this),
			boost::bind(&HeadArray::onCHold, this),
			boost::bind(&HeadArray::onCHold_release, this));

	leftBtn = new Button(left_pin, NULL,NULL,NULL,
			boost::bind(&HeadArray::onLPress, this),
			boost::bind(&HeadArray::onLRelease, this));
	rightBtn = new Button(right_pin,NULL,NULL,NULL,
			boost::bind(&HeadArray::onRPress, this),
			boost::bind(&HeadArray::onRRelease, this));

}

HeadArray::~HeadArray() {
	delete (this->statetimer);
	delete (this->centreBtn);
	delete (this->rightBtn);
	delete (this->leftBtn);
}

void HeadArray::start() {
	v=w= 0;
	forwardToggle = false;
	command_pub = n.advertise < geometry_msgs::TwistStamped > (this->topic, 10);
	neutralSA();
	centreBtn->start();
	rightBtn->start();
	leftBtn->start();
}

void HeadArray::stop() {
	this->statetimer->cancel();
	this->state = suspend;
	this->command_pub.shutdown();
	event = nill;

	centreBtn->stop();
	rightBtn->stop();
	leftBtn->stop();
}

void HeadArray::reset() {
	this->stop();
	this->start();
}

// Event callbacks for Centre button
void HeadArray::onCQuickpress() {
	this->statetimer->cancel();
	this->event = quickpress;
	this->linearTransition();
}

void HeadArray::onCHold_release() {
	this->statetimer->cancel();
	this->event = hold_release;
	this->linearTransition();
}

void HeadArray::onCHold() {
	this->statetimer->cancel();
	this->event = hold;
	this->linearTransition();
}
// **********Event callbacks for Right button ********

void HeadArray::onRPress() {
		this->w = 1;
		sendcommands();
}
void HeadArray::onRRelease() {
	this->w =0;
	sendcommands();
}
void HeadArray::onLPress() {
		this->w = -1;
		sendcommands();
}
void HeadArray::onLRelease() {
	this->w =0;
	sendcommands();
}

// **********State entry actions ********
void HeadArray::neutralSA() {
	this->state = neutral;
	this->neutral_time = gettime_ms();
	this->v = 0;
	sendcommands();
	ROS_INFO("ha neutral state");
}

void HeadArray::moveSA() {
	this->state = move;
	if (this->forwardToggle) {
		//move backwards
		this->v = -1;
	} else {
		this->v = 1;
	}
	sendcommands();
	ROS_INFO("ha move state");
}


void HeadArray::cToggle() {
	this->forwardToggle =! this->forwardToggle;
}
// in charge of sending commands to ensure that wheelchair will always
// turn in spot when only direction commands are given.
void HeadArray::sendcommands() {
	//
	geometry_msgs::TwistStamped cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "HA";

	cmd.twist.linear.x = this->v;
	cmd.twist.angular.z = this->w;

	this->command_pub.publish(cmd);
}

void HeadArray::linearTransition() {
	switch (this->state) {
	case neutral:
		if (this->event == hold) {
			this->state = move;
			moveSA();
		} else if (this->event == quickpress) {
			// Check if in neutral state for a minimum time to cancel jerk
			if ((gettime_ms() - this->neutral_time) > THRESHOLD / 2) {
				cToggle();
			}
		}
		break;
	case move:
		if (this->event == hold_release) {
			neutralSA();
		}
		break;
	default:
		neutralSA();
		break;
	}
}
