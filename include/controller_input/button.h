#ifndef BUTTON_H_
#define BUTTON_H_
#include <unistd.h>
#include<stdlib.h>
#include <sys/time.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "controller_input/gpio.h"
#include "controller_input/statetimer.h"



using namespace std;
using namespace exploringBB;





/***********************/
class Button {
// TODO: Add a suspend function button.

public:
	Button(int pin_t, boost::function<void (void)> qpcb=NULL,
			boost::function<void (void)> hcb=NULL, boost::function<void (void)> hrcb=NULL,
			boost::function<void (void)> pcb=NULL,
			boost::function<void (void)> rcb=NULL);
	~Button();
	void set_quickpress_duration(int dur) {
			quickpress_duration = dur;
		}
	void start();
	void stop();

public:
	typedef enum {neutral_0, timing, hold, suspend} ButtonState;
	typedef enum {risingedge, fallingedge, exceedthres, None} ButtonEvent;
private:

	myGPIO* pin;
	ButtonEvent event;
	ButtonState state;
	StateTimer *stateTimer;
	int quickpress_duration;
	// Event Handlers
	void onRisingEdge();
	void onFallingEdge();
	void onExceedThreshold();
	void transition();
	void initialise(int pin);

	// State Entry Actions
	void timingSA();
	void neutralSA();
	void holdSA();


	//Actions
	void quickPressAction();
	void pressAction();
	void hold_releaseAction();
	void releaseAction();
	//external callbacks for internal states
	boost::function<void (void)> pressCallback;
	boost::function<void (void)> quickpressCallback;
	boost::function<void (void)> holdCallback;
	boost::function<void (void)> hold_releaseCallback;
	boost::function<void (void)> releaseCallback;


};


#endif
