#include "controller_input/joystick.h"
#define AIN_PATH "/sys/devices/platform/ocp/44e0d000.tscadc/TI-am335x-adc/iio:device0/in_voltage"

#define equals(x, y) (abs(x-y) < 0.008)

using namespace std;


Joystick::Joystick(int speedpin, int dirpin, const char * topic, ros::NodeHandle &n_t) {
	this->speed_ain = speedpin;
	this->dir_ain = dirpin;

	this->n = n_t;
	this->topic = topic;

	this->isrunning = false;
	pthread_attr_init(&this->tattr);
	pthread_attr_setdetachstate(&this->tattr,PTHREAD_CREATE_DETACHED);
}

int Joystick::start(int rate) {
	// read
	this->rate = rate;
	this->isrunning = true;
	command_pub = n.advertise < geometry_msgs::TwistStamped > (this->topic, 10);
	// analog read polling started in new thread
	if (pthread_create(&this->thread, &this->tattr, &JSthreadedPoll,
			static_cast<void*>(this))) {
		perror("Joystick: Failed to create the poll thread");
		this->isrunning = false;
		return -1;
	}
	return 0;
}

void Joystick::pollOnce() {
	float v = readAnalog(this->speed_ain);
	float w = readAnalog(this->dir_ain);
	cout << "[JS] v=" << v << " ,w=" << w << endl;
	this->cmd.header.stamp = ros::Time::now();
	this->cmd.twist.linear.x = v;
	this->cmd.twist.angular.z = w;
	this->command_pub.publish(this->cmd);
}

void* JSthreadedPoll(void *val) {

	Joystick *js = static_cast<Joystick*>(val);
	int rate = 1000000/js->rate;
	cout <<"[JS] rate=" <<rate <<endl;
	while (js->isrunning) {
		js->pollOnce();
		usleep(rate); // ~20Hz
	}
	return 0;
}

void Joystick::stop() {
	this->isrunning = false;
	this->command_pub.shutdown();
}

float Joystick::readAnalog(int number) {
	stringstream ss;
	ss << AIN_PATH << number << "_raw";
	fstream fs;
	fs.open(ss.str().c_str(), fstream::in);
	fs >> number;
	fs.close();

	/*
	 * 2.41V from joystick is read as 1707.
	 * Thus from -1 to +1 ofmax speed gives
	 * 5V = 2 then 2.41V = ? given that 2.41 = 1707
	 * We get  ratio of max val = -1 + (analogread)* 0.0005647334505
	 */

	/*
	 * max = 2768
	 * neutral = 1707
	 * min = 580
	 */
	float ret;
	if (number > 1707) {
		// 1100 = 1
		ret = float(number-1707) * 0.0004545454545;//0.0009090909091 *0.5; //*1/(2768-1707)
	} else {
		// 1180
		ret = float(number-1707) * 0.0004237288136; //0.0008474576271 *0.5; //*1/(2768-1707)
	}
	return equals(ret,0)? 0:ret;
}
