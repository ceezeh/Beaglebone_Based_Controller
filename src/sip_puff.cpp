#include "controller_input/sip_puff.h"
#define equals(x, y) (abs(x-y) < 0.001)

using namespace std;

using namespace exploringBB;

// TODO: INC and DEC may need angula speeds that map straight to or correspond with linear speeds.
Sippuff::Sippuff(int sip_pin, int puff_pin, const char * topic,
		ros::NodeHandle &n_t) {
	n = n_t;
	this->topic = topic;


	sipBtn = new Button(sip_pin, boost::bind(&Sippuff::onSQuickpress, this),
			boost::bind(&Sippuff::onSHold, this),
			boost::bind(&Sippuff::onSHold_release, this));

	puffBtn = new Button(puff_pin, boost::bind(&Sippuff::onPQuickpress, this),
			boost::bind(&Sippuff::onPHold, this),
			boost::bind(&Sippuff::onPHold_release, this));

}
Sippuff::~Sippuff() {
	delete (this->sipBtn);
	delete (this->puffBtn);
}
void Sippuff::start() {
	w = v = 0;
	command_pub = n.advertise < geometry_msgs::TwistStamped > (this->topic, 10);
	sipBtn->start();
	puffBtn->start();
}

void Sippuff::stop() {
	this->command_pub.shutdown();
	sipBtn->stop();
	puffBtn->stop();
}

void Sippuff::reset() {
	this->stop();
	this->start();
}

void Sippuff::decrementSpeed() {
	this->v--;
	if (v < -1)
		v = -1;
	sendcommands();
}

void Sippuff::incrementSpeed() {
	this->v++;
	if (v > 1)
		v = 1;
	sendcommands();
}

void Sippuff::turnLeft() {
	this->w = -1;
	sendcommands();
}
void Sippuff::turnRight() {
	this->w = 1;
	sendcommands();
}
void Sippuff::stopTurn() {
	this->w = 0;
	sendcommands();
}

void Sippuff::stopMove() {
	this->v = 0;
	sendcommands();
}

/********************Event Callbacks *****************************/
void Sippuff::onSQuickpress() {
	this->decrementSpeed();
}

void Sippuff::onSHold() {
	this->turnLeft();
}

void Sippuff::onSHold_release() {
	this->stopTurn();
}

// Callbacks for Puff buttons.
void Sippuff::onPQuickpress() {
	this->incrementSpeed();
}

void Sippuff::onPHold() {
	this->turnRight();
}

void Sippuff::onPHold_release() {
	this->stopTurn();
}

/**********************/
// in charge of sending commands to ensure that wheelchair will always
// turn in spot when only direction commands are given.
void Sippuff::sendcommands() {
	geometry_msgs::TwistStamped cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "SP";
	cmd.twist.angular.z = this->w;
	cmd.twist.linear.x = this->v;
	this->command_pub.publish(cmd);
}
