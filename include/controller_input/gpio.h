#ifndef MYGPIO_H_
#define MYGPIO_H_
#include "exploringBB/gpio/GPIO.h"
#include "ros/ros.h"
#include <sys/epoll.h>
#include <fcntl.h>
#include <unistd.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <mutex>

using namespace std;
using namespace exploringBB;
void* mythreadedPoll(void *value);

/*
 * We have two requirements for our gpio class.
 * First, we need to specify timeout for pin listening.
 * Second, we need to implement an arbitrary callback.
 */

class myGPIO: public GPIO{
public:
	using GPIO::GPIO;
	using GPIO::DIRECTION;
	using GPIO::VALUE;
	using GPIO::EDGE;

	int waitForEdge(boost::function<void(void)> callback, int duration);

	boost::function<void(void)> callback;
	myGPIO(int number);
private:

	using GPIO::threadRunning;
	friend void* mythreadedPoll(void *value);
	int _waitForEdge(int duration=-1);
	int duration;
	pthread_attr_t tattr;

	int epollfd;
//	struct epoll_event ev;
	std::mutex m;
};
#endif
