#include "controller_input/statetimer.h"

using namespace std;
using namespace exploringBB;


StateTimer::StateTimer() {
	threadRunning =  false;
	pthread_attr_init(&this->tattr);
	pthread_attr_setdetachstate(&this->tattr,PTHREAD_CREATE_DETACHED);
}

int StateTimer::start(boost::function<void (void)> callback, int dur) { //duration is in microseconds.
	this->callback = callback;

	struct_st_args* args = (struct_st_args*)malloc(sizeof(struct_st_args));
	args->t = dur;
	args->st = this;
	this->threadRunning =  true;
	if(pthread_create(&this->thread, &this->tattr, &STthreadedPoll, static_cast<void*>(args))){
		perror("StateTimer Failed to create the poll thread");
		this->threadRunning = false;
		return -1;
	}
	return 0;
}
///*
// * This function polls the time until time elapsed or timer is stopped.
// */
void* STthreadedPoll(void *val){
	StateTimer::struct_st_args* args  = static_cast<StateTimer::struct_st_args*>(val);
	int dur = args->t;
	StateTimer *st = args->st;
	timestamp_t starttime = gettime_ms(); //Microseconds.

	while (st->threadRunning){

		timestamp_t  now =  gettime_ms();
		int elapsed = (int) (now - starttime);
		if (elapsed > dur) {
			((st->callback))();
			st->threadRunning = false;
		}
		usleep(10000);
	}
	free (args);
	return 0;
}
//
void StateTimer::cancel() {
	this->threadRunning = false;
	this->callback = NULL;
}
/**********/
timestamp_t gettime_ms() {
	struct timeval tp;
	gettimeofday(&tp, NULL);
	timestamp_t now = (long long) tp.tv_sec * 1000L + tp.tv_usec/1000;
	return now;
}
