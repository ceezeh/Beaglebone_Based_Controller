#include "controller_input/sip_puff.h"
#define THRESHOLD 500 //ms
#define INVALIDCMD -2
#define SPEEDINCR 0.2
#define DIRINCR 0.1
#define equals(x, y) (abs(x-y) < 0.001)
#define LINSPEED 0.7
#define ANGSPEED 0.5
#define TURNSPEED 0.05
using namespace std;

using namespace exploringBB;

// TODO: INC and DEC may need angula speeds that map straight to or correspond with linear speeds.
Sippuff::Sippuff(int sip_pin, int puff_pin, const char * topic,
		ros::NodeHandle &n_t) {
	n = n_t;
	this->topic = topic;
	statetimer = new StateTimer();

	threshold = THRESHOLD;

	sipBtn = new Button(sip_pin, boost::bind(&Sippuff::onSQuickpress, this),
			boost::bind(&Sippuff::onSHold, this),
			boost::bind(&Sippuff::onSHold_release, this));

	puffBtn = new Button(puff_pin, boost::bind(&Sippuff::onPQuickpress, this),
			boost::bind(&Sippuff::onPHold, this),
			boost::bind(&Sippuff::onPHold_release, this));

}
Sippuff::~Sippuff() {
	delete (this->statetimer);
	delete (this->sipBtn);
	delete (this->puffBtn);
}
void Sippuff::start() {
	prev_v = prev_w= w= v=0;

	this->prev_state = neutral;
	this->state = neutral;
	command_pub = n.advertise < geometry_msgs::TwistStamped > (this->topic, 10);
	neutralSA();
	sipBtn->start();
	puffBtn->start();
}

void Sippuff::stop() {
	this->statetimer->cancel();
	this->state = suspend;
	this->command_pub.shutdown();
	event = nill;

	sipBtn->stop();
	puffBtn->stop();
}

void Sippuff::reset() {
	this->stop();
	this->start();
}

/********************State Actions ***********************/
void Sippuff::neutralSA() {
	this->state = neutral;
	this->prev_state = neutral;
	setw(0.7* LINSPEED);
	setv(LINSPEED);
	sendcommands(0, INVALIDCMD);
	cout << "[SP] neutral state" << endl;

}
void Sippuff::goslowSA(ButtonType btn) {
	this->state = goslow;
	if (btn == SIP) {
		sendcommands(-this->slowspeed[0], INVALIDCMD);
	} else if (btn == PUFF) {
		sendcommands(this->slowspeed[0]+0.005, INVALIDCMD);
		cout << "[SP] PUFF" << endl;
	}
	cout << "[SP] go slow" << endl;
}
void Sippuff::timings0SA() {
	this->state = timings0;
	this->statetimer->start(boost::bind(&Sippuff::onSExceedThres, this),
			this->threshold);
	cout << "[SP] timings0 state" << endl;
}
void Sippuff::timings1SA() {
	this->state = timings1;
	this->statetimer->start(boost::bind(&Sippuff::onSExceedThres, this),
			this->threshold);
	cout << "[SP] timings1 state" << endl;
}
void Sippuff::backwardSA() {
	this->state = backward;
	this->prev_state = backward;
	sendcommands(-this->v, INVALIDCMD);
	cout << "[SP] backward state; v = " << this->v << endl;
}
void Sippuff::decrementSA() {
	cout << "[SP] decrement state" << endl;
	this->state = decrement;
	sendcommands(this->prev_v -= SPEEDINCR, 0);
	// bootstrapping w value to v value.
	setv(this->prev_v);
	setw(abs(this->prev_v *.7));
	// perform action.
	backwardSA();
}

void Sippuff::timingp0SA() {
	this->state = timingp0;
	this->statetimer->start(boost::bind(&Sippuff::onPExceedThres, this),
			this->threshold);
	cout << "[SP] timingp0 state" << endl;
}
void Sippuff::timingp1SA() {
	this->state = timingp1;
	this->statetimer->start(boost::bind(&Sippuff::onPExceedThres, this),
			this->threshold);
	cout << "[SP] timingp1 state" << endl;
}
void Sippuff::forwardSA() {
	this->state = forward;
	this->prev_state = forward;
	sendcommands(this->v, INVALIDCMD);
	cout << "[SP] forward state" << endl;
}
void Sippuff::incrementSA() {
	this->state = increment;
	cout << "[SP] increment state" << endl;
	// perform action.
	sendcommands(this->prev_v += SPEEDINCR, 0);
	// bootstrapping w value to v value.
	setv(this->prev_v);
	setw(abs(this->prev_v *.7));
	forwardSA();
}

void Sippuff::leftSA() {
	this->state = left;
	sendcommands(INVALIDCMD, -this->w);
	cout << "[SP] left state" << endl;
}
void Sippuff::rightSA() {
	this->state = right;
	sendcommands(INVALIDCMD, this->w);
	cout << "[SP] right state" << endl;
}
void Sippuff::rightbackwardSA() {
	this->state = rightbackward;
	sendcommands(INVALIDCMD, this->w);
	cout << "[SP] right backward state" << endl;
}
void Sippuff::leftbackwardSA() {
	this->state = leftbackward;
	sendcommands(INVALIDCMD, -this->w);
	cout << "[SP] left backward state" << endl;
}
void Sippuff::rightforwardSA() {
	this->state = rightforward;
	sendcommands(INVALIDCMD, this->w);
	cout << "[SP] right forward state" << endl;
}
void Sippuff::leftforwardSA() {
	this->state = leftforward;
	sendcommands(INVALIDCMD, -this->w);
	cout << "[SP] left forward state" << endl;
}
/********************Event Callbacks *****************************/
void Sippuff::onSQuickpress() {
	if ((this->state == timings1) || (this->state == timingp1))
		return;
	if (this->state == timingp0)
		return;

	this->statetimer->cancel();
	cout << "[SP] squickpress event" << endl;
	this->event = squickpress;
	this->transition();
}

void Sippuff::onSHold() {
	if ((this->state == timingp0) || (this->state == timingp1))
		return;
	this->statetimer->cancel();
	cout << "[SP] shold event" << endl;
	this->event = shold;
	this->transition();
}

void Sippuff::onSExceedThres() {
	this->statetimer->cancel();
	cout << "[SP] sexceed thres event" << endl;
	this->event = sthres;
	this->transition();
}

void Sippuff::onSHold_release() {
	if ((this->state == timingp0) || (this->state == timingp1))
		return;
	this->statetimer->cancel();
	cout << "[SP] shold_release event" << endl;
	this->event = shold_release;
	this->transition();
}

// Callbacks for Puff buttons.
void Sippuff::onPQuickpress() {
	// this is a patch
	if ((this->state == timings1) || (this->state == timingp1))
		return;
	if (this->state == timings0)
		return;
	this->statetimer->cancel();

	cout << "[SP] pquickpress event" << endl;
	this->event = pquickpress;
	this->transition();
}

void Sippuff::onPHold() {
	if ((this->state == timings0) || (this->state == timings1))
		return;
	this->statetimer->cancel();
	cout << "[SP] phold event" << endl;
	this->event = phold;
	this->transition();
}

void Sippuff::onPExceedThres() {
	this->statetimer->cancel();
	cout << "[SP] pexceedthres event" << endl;
	this->event = pthres;
	this->transition();
}

void Sippuff::onPHold_release() {
	if ((this->state == timings0) || (this->state == timings1))
		return;
	this->statetimer->cancel();
	cout << "[SP] phold_release event" << endl;
	this->event = phold_release;
	this->transition();
}

void Sippuff::transition() {
	switch (this->state) {
	case neutral:
		if (this->event == squickpress) {
			timings0SA();
//			this->state = timings0;
//			this->statetimer->start(boost::bind(&Sippuff::onSExceedThres, this),
//					this->threshold);
		} else if (this->event == pquickpress) {
			timingp0SA();
//			this->state = timingp0;
//			this->statetimer->start(boost::bind(&Sippuff::onPExceedThres, this),
//					this->threshold);
		} else if (this->event == shold) {
			this->state = left;
			leftSA();
		} else if (this->event == phold) {
			this->state = right;
			rightSA();
		}
		break;
	case left:
	case right:
	case leftbackward:
	case rightbackward:
	case leftforward:
	case rightforward:
		if ((this->event == shold_release) || (this->event == phold_release)) {
			if (this->prev_state == forward) {
				this->state = forward;
				forwardSA();
			} else if (this->prev_state == backward) {
				backwardSA();
			} else if (this->prev_state == neutral) {
				neutralSA();
			}
		}
		break;
	case timings0:
		if (this->event == squickpress) {
			timings1SA();
//			this->state = timings1;
//			this->statetimer->start(boost::bind(&Sippuff::onSExceedThres, this),
//					this->threshold);
		} else if (this->event == shold) {
//			this->state = goslow;
			goslowSA(SIP);
		} else if (this->event == sthres) {
			if (this->prev_state == backward) {
//				this->state = neutral;
//				this->prev_state = neutral;
				neutralSA();
			} else if (this->prev_state == neutral) {
				backwardSA();
//				this->state = backward;
//				this->prev_state = backward;
			}
		} else if (this->event == pthres) {
//			this->state = neutral;
//			this->prev_state = neutral;
			neutralSA();
		}
		break;
	case timings1:
		if (this->event == sthres) {
			decrementSA();
//			this->state = decrement;
//			// perform action.
//			this->state = neutral;
//			this->prev_state = neutral;

		} else if (this->event == shold) {
//			this->state = goslow;
			goslowSA(SIP);
		}
		break;
	case backward:
		if ((this->event == squickpress) || (this->event == pquickpress)) {
//			this->state = timings0;
			timings0SA();
		} else if (this->event == shold) {
			this->state = leftbackward;
			leftbackwardSA();
		} else if (this->event == phold) {
			this->state = rightbackward;
			rightbackwardSA();
		}
		break;
	case goslow:
		if ((this->event == shold_release) || (this->event == phold_release)) {
//			this->state = neutral;
			neutralSA();
		}
		break;
	case timingp0:
		if (this->event == pquickpress) {
			this->state = timingp1;
			timingp1SA();
		} else if (this->event == phold) {
			this->state = goslow;
			goslowSA(PUFF);
		} else if (this->event == pthres) {
			if (this->prev_state == forward) {
				neutralSA();
//				this->state = neutral;
//				this->prev_state = neutral;
			} else if (this->prev_state == neutral) {
//				this->state = forward;
//				this->prev_state = forward;
				forwardSA();
			}
		} else if (this->event == sthres) {
//			this->state = neutral;
//			this->prev_state = neutral;
			neutralSA();
		}
		break;
	case timingp1:
		if (this->event == pthres) {
			incrementSA();
//			this->state = increment;
//			// perform action.
//			this->state = neutral;
//			this->prev_state = neutral;
		} else if (this->event == phold) {
//			this->state = goslow;
			goslowSA(PUFF);
		}
		break;
	case forward:
		if ((this->event == squickpress) || (this->event == pquickpress)) {
			this->state = timingp0;
			timingp0SA();
		} else if (this->event == shold) {
			this->state = leftforward;
			leftforwardSA();

		} else if (this->event == phold) {
			this->state = rightforward;
			rightforwardSA();
		}
		break;
	default:
		break;
	}
}
/**********************/
// in charge of sending commands to ensure that wheelchair will always
// turn in spot when only direction commands are given.
void Sippuff::sendcommands(float vt, float wt) {
	//
	this->cmd.header.stamp = ros::Time::now();
	this->cmd.header.frame_id = "sippuff";
#define V_BIT (1 << 0)
#define W_BIT (1 << 1)

	switch ((equals(prev_v, 0) ? 0 : V_BIT) | (equals(prev_w, 0) ? 0 : W_BIT)) {
	case 0: //WC is not moving
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = vt;
			this->cmd.twist.angular.z = 0;
			this->prev_v = vt;
			this->prev_w = 0;
			cout << "1" << endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_v = 0;
			this->prev_w = 0;
			cout << "2" << endl;
		} else if ((!equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = TURNSPEED;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			cout << "3" << endl;
		} else if ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_v = 0;
			this->prev_w = 0;
			cout << "4" << endl;
		}
		break;
	case V_BIT: // V is non-stationary and W is stationary
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = vt;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = vt;
			cout << "5" << endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_v = 0;
			this->prev_w = 0;
			cout << "6" << endl;
		} else if ((!equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			cout << "7" << endl;
		} else if ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = 0;
			this->prev_w = 0;
			cout << "8" << endl;
		}
		break;
	case W_BIT: // V is stationary and W is non-stationary
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)) { //invalid case
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_v = vt;
			this->prev_w = 0;
			cout << "9" << endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) { //invalid case
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_v = 0;
			this->prev_w = 0;
			cout << "10" << endl;
		} else if ((!equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			cout << "11" << endl;
		} else if ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_w = 0;
			this->prev_v = 0;
			cout << "12" << endl;
		}
		break;
	case V_BIT + W_BIT: // V is non-stationary and W is non-stationary
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = vt;
			this->cmd.twist.angular.z = 0;
			this->prev_v = vt;
			this->prev_w = 0;
			cout << "13" << endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) { //
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_v = 0;
			this->prev_w = 0;
			cout << "14" << endl;
		} else if ((!equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			cout << "15" << endl;
		} else if ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = 0;
			this->prev_w = 0;
			cout << "16" << endl;
		}
		break;
	}
	this->cmd.twist.angular.z = (equals(this->cmd.twist.angular.z,INVALIDCMD))? this->cmd.twist.angular.z:-this->cmd.twist.angular.z;
	this->command_pub.publish(this->cmd);
}
