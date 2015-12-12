#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_

#include <cstdio>
#include <unistd.h>
#include <sys/time.h>
typedef unsigned long long timestamp_t;

extern timestamp_t gettime_ms();

#endif

#ifndef _STATETIMER
#define _STATETIMER


#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "controller_input/gpio.h"



using namespace std;
using namespace exploringBB;


/***********************/
void* STthreadedPoll(void *val);

/***********************/
class StateTimer{

public:
	typedef struct {StateTimer *st; int t; } struct_st_args;

	StateTimer();
	void cancel();
	int start(boost::function<void (void)> callback, int dur);


private:

	boost::function<void (void)> callback;
	bool threadRunning;
	pthread_t thread;
	friend void* STthreadedPoll(void *val);
	pthread_attr_t tattr;
};
#endif
