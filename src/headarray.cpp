#include "controller_input/headarray.h"
#define THRESHOLD 300 //ms
#define INVALIDCMD -2
#define SPEEDINCR 0.05
#define DIRINCR 0.05
#define equals(x, y) (abs(x-y) < 0.001)
#define LINSPEED 0.2
#define ANGSPEED 0.2
#define TURNSPEED 0.05
using namespace std;

using namespace exploringBB;

HeadArray::HeadArray(int left_pin, int centre_pin, int right_pin, const char * topic, ros::NodeHandle &n_t ) {
	n =n_t;
	this->topic= topic;
//	command_pub = n.advertise<geometry_msgs::Twist>( this->topic, 10);

//	v=w= 0;
//	forwardToggle = false;

	statetimer = new StateTimer();
//	event = nill;
//	neutral_reset = true;
	threshold = THRESHOLD;
	// enter neutral state

	centreBtn = new Button(centre_pin, boost::bind( &HeadArray::onCQuickpress, this),
			boost::bind( &HeadArray::onCHold, this),
	boost::bind( &HeadArray::onCHold_release, this));

	leftBtn = new Button(left_pin, boost::bind( &HeadArray::onLQuickpress, this),
			boost::bind( &HeadArray::onLHold, this),
	boost::bind( &HeadArray::onLHold_release, this));
	rightBtn = new Button(right_pin, boost::bind( &HeadArray::onRQuickpress, this),
			boost::bind( &HeadArray::onRHold, this),
	boost::bind( &HeadArray::onRHold_release, this));

}

HeadArray::~HeadArray() {
	delete(this->statetimer);
	delete(this->centreBtn);
	delete(this->rightBtn);
	delete(this->leftBtn);
}

void HeadArray::start() {
	prev_v = prev_w= w= 0;
	forwardToggle = false;
	command_pub = n.advertise<geometry_msgs::TwistStamped>(this->topic, 10);
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
	this->transition();
}

void HeadArray::onCHold_release() {
	this->statetimer->cancel();
	this->event = hold_release;
	this->transition();
}

void HeadArray::onCHold() {
	this->statetimer->cancel();
	this->event = hold;
	this->transition();
}

void HeadArray::onCExceedThres() {
	this->statetimer->cancel();
	this->event = thres;
	this->transition();
}
// **********Event callbacks for Right button ********
void HeadArray::onRQuickpress() {
	if ((this->state == move)||(this->state == goslow)) {
		if (this->forwardToggle) {
			sendcommands(this->prev_v-=SPEEDINCR, INVALIDCMD);
//			update rotation speed since we won't use it yet. Do we need to slow down rotation speed?
			setw(this->w -= DIRINCR);
		} else {
			sendcommands(this->prev_v+=SPEEDINCR, INVALIDCMD);
			//update rotation speed since we won't use it yet. Do we need to slow down rotation speed?

			setw(this->w  += DIRINCR);
		}
	}
}
void HeadArray::onRHold() {
	if ((this->state == move) ||(this->state == neutral)){
		/* Send the previous speed if it was not zero.
		 * This lets us keep the previous speed setting until we go back to neutral position
		 * where everything resets.
		 */

		sendcommands(INVALIDCMD,this->w);

	} else if (this->state == goslow) {
		sendcommands(INVALIDCMD,this->slowspeed[1]);
	}
}
void HeadArray::onRHold_release() {
	sendcommands(INVALIDCMD,0);
}
// **********Event callbacks for Right button ********
void HeadArray::onLQuickpress() {
	if ((this->state == move)||(this->state == goslow)) {
		if (this->forwardToggle) {
			sendcommands(this->prev_v+=SPEEDINCR, INVALIDCMD);
//			update rotation speed since we won't use it yet. Do we need to slow down rotation speed?
			setw(this->w += DIRINCR);
		} else {
			sendcommands(this->prev_v-=SPEEDINCR, INVALIDCMD);
			//update rotation speed since we won't use it yet. Do we need to slow down rotation speed?

			setw(this->w  -= DIRINCR);
		}
	}
}
void HeadArray::onLHold() {
	if ((this->state == move) ||(this->state == neutral)){
		/* Send the previous speed if it was not zero.
		 * This lets us keep the previous speed setting until we go back to neutral position
		 * where everything resets.
		 */

		sendcommands(INVALIDCMD,-this->w);

	} else if (this->state == goslow) {
		sendcommands(INVALIDCMD,-this->slowspeed[1]);
	}
}
void HeadArray::onLHold_release() {
	sendcommands(INVALIDCMD,0);
}

// **********State entry actions ********
void HeadArray::neutralSA() {
	setw(ANGSPEED); // next time rotation is called it will start from default speed.
	this->state = neutral;
	this->neutral_time = gettime_ms();
	sendcommands(0, INVALIDCMD);
	// cout <<"ha neutral state" <<endl;
	//reset cmds.
}

void HeadArray::goslowSA() {
	//CHECK which button entered this state.
	setw(this->slowspeed[1]);
	if (this->forwardToggle) {
		sendcommands(-this->slowspeed[0], INVALIDCMD);
	} else {
		sendcommands(this->slowspeed[0], INVALIDCMD);
	}
}

// in charge of sending commands to ensure that wheelchair will always
// turn in spot when only direction commands are given.
void HeadArray::sendcommands(float vt, float wt) {
	//
	this->cmd.header.stamp = ros::Time::now();

//	// Check if last command was stationary linear velocity
//	if (abs(this->prev_v) < 0.001) {
//		if (requestTurn){
//			this->cmd.twist.linear.x = 0.1;
//			this->cmd.twist.angular.z = wt;
////			this->w =wt;
//		}
//	} else {
//		if (requestTurn){
//			this->cmd.twist.linear.x = INVALIDCMD;
//			this->cmd.twist.angular.z = wt;
////			this->w = wt;
//		}
//	}
//	if (!requestTurn) {
//		// When we dont want to translate but we were rotating
//		if ((abs(vt) < 0.001) && (abs(cmd.twist.angular.z) >0.001)){
//			this->cmd.twist.linear.x = 0.1;
//		} else if( (((abs(this->prev_v) < 0.001) && (vt == INVALIDCMD )) ||
//				(abs(vt) < 0.001)) &&
//				(abs(wt) < 0.001)){ // When we dont want to translate and do not want to rotate.
//			this->cmd.twist.linear.x = 0;
//			this->prev_v = vt;
//		} else {
//			this->cmd.twist.linear.x = vt;
//			this->prev_v = vt;
//		}
//
//		this->cmd.twist.angular.z =  INVALIDCMD;
////		this->v =vt;
//	}
#define V_BIT (1 << 0)
#define W_BIT (1 << 1)

	switch( (equals(prev_v, 0)? 0: V_BIT ) | (equals(prev_w, 0)? 0 :W_BIT)) {
	case 0: //WC is not moving
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)){
			this->cmd.twist.linear.x = vt;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = vt;
			// cout <<"1"<<endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = 0;
			// cout <<"2"<<endl;
		} else if ((!equals(wt, 0)) && equals(vt, INVALIDCMD)){
			this->cmd.twist.linear.x = TURNSPEED;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			// cout <<"3"<<endl;
		} else if  ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_w = 0;
			// cout <<"4"<<endl;
		}
		break;
	case V_BIT: // V is non-stationary and W is stationary
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)){
			this->cmd.twist.linear.x = vt;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = vt;
			// cout <<"5"<<endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = 0;
			// cout <<"6"<<endl;
		} else if  ((!equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			// cout <<"7"<<endl;
		} else if ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = 0;
			this->prev_w = 0;
			// cout <<"8"<<endl;
		}
		break;
	case W_BIT: // V is stationary and W is non-stationary
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)){
			this->cmd.twist.linear.x = vt;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = vt;
			// cout <<"9"<<endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = TURNSPEED;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = 0;
			// cout <<"10"<<endl;
		} else if ((!equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			// cout <<"11"<<endl;
		} else if ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = 0;
			this->cmd.twist.angular.z = 0;
			this->prev_w = 0;
			// cout <<"12"<<endl;
		}
		break;
	case V_BIT + W_BIT: // V is non-stationary and W is non-stationary
		if ((!equals(vt, 0)) && equals(wt, INVALIDCMD)){
			this->cmd.twist.linear.x = vt;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = vt;
			// cout <<"13"<<endl;
		} else if ((equals(vt, 0)) && equals(wt, INVALIDCMD)) {
			this->cmd.twist.linear.x = TURNSPEED;
			this->cmd.twist.angular.z = INVALIDCMD;
			this->prev_v = 0;
			// cout <<"14"<<endl;
		} else if ((!equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = wt;
			this->prev_w = wt;
			// cout <<"15"<<endl;
		} else if ((equals(wt, 0)) && equals(vt, INVALIDCMD)) {
			this->cmd.twist.linear.x = INVALIDCMD;
			this->cmd.twist.angular.z = 0;
			this->prev_w = 0;
			// cout <<"16"<<endl;
		}
		break;
	}
	this->command_pub.publish(this->cmd);
}



void HeadArray::transition () {
	switch(this->state) {
	case neutral:
		if (this->event == hold) {
			this->state = move;
			// perform action
			if (this->forwardToggle) {
				sendcommands(-LINSPEED, INVALIDCMD);
			} else {
				sendcommands(LINSPEED, INVALIDCMD);
			}

			// cout <<"ha move state" <<endl;
		} else if (this->event == quickpress) {
			// Check if in neutral state for a minimum time to cancel jerk
			if ((gettime_ms() -this->neutral_time) > THRESHOLD/2) {
				this->state = timing;
				// start timing
				this->statetimer->start(boost::bind(&HeadArray::onCExceedThres, this), this->threshold);
				this->timing_time = gettime_ms();
				// cout <<"ha timing state" <<endl;
			}
		}
		break;
	case move:
		if (this->event == hold_release) {
			this->state = neutral;

			neutralSA();
		}
		break;
	case timing:
		if (this->event == hold) {
			timestamp_t now = gettime_ms();
			if ((now-this->timing_time) < THRESHOLD) {
				this->state = goslow;
				goslowSA();
				// cout <<"ha goslow state" <<endl;
			}
		} else if (this->event == thres) {
			this->state = toggle;
			//execute action and ...
			// cout <<" ha toggle state" <<endl;
			forwardToggle =!forwardToggle;
			// go to neutral
			neutralSA();
		}
		break;
	case goslow:
		if (this->event == hold_release){
			this->state = neutral;
			neutralSA();
		}
		break;
	case toggle:
		this->state = neutral;
		neutralSA();
		break;
	default:
			break;
	}
}
