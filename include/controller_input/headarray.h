#ifndef HEADARRAY_H_
#define HEADARRAY_H_
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
class HeadArray {

public:
	typedef enum {
		neutral, timing, toggle, goslow, move, suspend
	} State;
	typedef enum {
		nill, quickpress, hold, thres, hold_release
	} Event;

	const float slowspeed[2] = { 0.05, 0.05 }; //v,w
	/* TODO: create a state and event messages.
	 * States are the eight cardinal directions plus neutral.
	 * Events are state transitions: the increased/decreased speed, toggled mode change, going slow.
	 */

	HeadArray(int left_pin, int centre_pin, int right_pin, const char * topic,
			ros::NodeHandle &n_t);
	~HeadArray();
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
	float w; // w represents the velocity when moving so it is non zero.
	float prev_v, prev_w;
	// This is needed since we change w alongside v.

	void setw(float wt) {
		if (wt > 1)
			wt = 1;
		if (wt < 0)
			wt = 0;
		this->w = wt;
	}
	geometry_msgs::TwistStamped cmd;
	void sendcommands(float v, float w);
	bool forwardToggle;

	State state;
	Event event;
//    bool neutral_reset;

	Button* centreBtn, *leftBtn, *rightBtn;

	int threshold;

	void transition();
	StateTimer *statetimer;

	// Event callbacks for centre button.
	void onCQuickpress();
	void onCHold();
	void onCExceedThres();
	void onCHold_release();

	// Callbacks for left and right buttons.
	void onRQuickpress();
	void onRHold();
	void onRHold_release();

	void onLQuickpress();
	void onLHold();
	void onLHold_release();

	// State Entry Actions();
	timestamp_t neutral_time;
	timestamp_t timing_time;
	void neutralSA();
	void goslowSA();

};
#endif
