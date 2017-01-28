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
	Sippuff(int sip_pin, int puff_pin, const char * topic,
			ros::NodeHandle &n_t);
	~Sippuff();
	void stop(); // suspends the head-array interface.
	void start();
	void reset();
private:
	ros::Publisher command_pub;
	ros::NodeHandle n;
	const char * topic;
	/*these are ratios of the maximum speed that we will issue to the wheelchair.
	 * but that will change in different places.
	 */
	float w, v;

	void sendcommands();

	Button* sipBtn, *puffBtn;

	// Event callbacks for sip button.
	void onSQuickpress();
	void onSHold();
	void onSHold_release();

	// Callbacks for Puff buttons.
	void onPQuickpress();
	void onPHold();
	void onPHold_release();

	void decrementSpeed();
	void incrementSpeed();
	void turnLeft();
	void turnRight();
	void stopMove();
	void stopTurn();
};
#endif
