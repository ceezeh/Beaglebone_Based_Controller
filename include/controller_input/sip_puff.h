#ifndef SIPPUFF_H_
#define SIPPUFF_H_
#include "exploringBB/gpio/GPIO.h"
#include <unistd.h>
#include "controller_input/statetimer.h"
#include "controller_input/button.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"

using namespace std;
using namespace exploringBB;

// TODO implement complete state transition for left and right buttons.
class Sippuff {

public:
	typedef enum {
		neutral,
		timings0,
		timings1,
		timingp0,
		timingp1,
		goslow,
		forward,
		backward,
		increment,
		decrement,
		left,
		right,
		leftbackward,
		leftforward,
		rightbackward,
		rightforward,
		suspend
	} State;

	typedef enum {
		nill,
		squickpress,
		pquickpress,
		shold,
		phold,
		sthres,
		pthres,
		shold_release,
		phold_release
	} Event;

	typedef enum {
		SIP, PUFF
	} ButtonType;
	const float slowspeed[2] = { 0.05, 0.05 }; //v,w
	/* TODO: create a state and event messages.
	 * States are the eight cardinal directions plus neutral.
	 * Events are state transitions: the increased/decreased speed, toggled mode change, going slow.
	 */

	Sippuff(int sip_pin, int puff_pin, const char * topic,
			ros::NodeHandle &n_t);
	~Sippuff();
	void stop(); // suspends the head-array interface.
	void start();
	void reset();
	void setThreshold(int dur);
private:
	ros::Publisher command_pub;
	ros::NodeHandle n;
	const char * topic;
	/*these are ratios of the maximum speed that we will issue to the wheelchair.
	 * but that will change in different places.
	 */
	float w,v; // w represents the velocity when moving so it is non zero.
	float prev_v, prev_w;
	// This is needed since we change w alongside v.
	void setv(float vt) {
		if (vt > 1)
			vt = 1;
		if (vt < -1)
			vt = -1;
		this->v = vt;
	}
	void setw(float wt) {
		if (wt > 1)
			wt = 1;
		if (wt < -1)
			wt = -1;
		this->w = wt;
	}
	geometry_msgs::TwistStamped cmd;
	void sendcommands(float v, float w);

	State state;
	/*
	 * Used to track directionality in timing0 state.
	 * This is useful for toggling between moving and stopping upon quickpress.
	 */
	State prev_state;
	Event event;

//    bool neutral_reset;

	Button* sipBtn, *puffBtn;

	int threshold;

	void transition();
	StateTimer *statetimer;

	// Event callbacks for sip button.
	void onSQuickpress();
	void onSHold();
	void onSExceedThres();
	void onSHold_release();

	// Callbacks for Puff buttons.
	void onPQuickpress();
	void onPHold();
	void onPExceedThres();
	void onPHold_release();

	// State Entry Actions();
	timestamp_t neutral_time;
	timestamp_t timing_time;
	void neutralSA();
	void goslowSA(ButtonType btn);
	void timings0SA();
	void timings1SA();
	void backwardSA();
	void decrementSA();

	void timingp0SA();
	void timingp1SA();
	void forwardSA();
	void incrementSA();

	void leftSA();
	void rightSA();
	void rightbackwardSA();
	void leftbackwardSA();
	void rightforwardSA();
	void leftforwardSA();
};
#endif
