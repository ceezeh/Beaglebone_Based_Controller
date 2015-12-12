#include "controller_input/button.h"

#define QUICKPRESS_DUR 300 // 100ms


using namespace std;
using namespace exploringBB;

//
//
//
///******************************************************************/

Button::Button (int pin_t, boost::function<void (void)> qpcb, boost::function<void (void)> hcb, boost::function<void (void)> hrcb) {
	this->holdCallback = hcb;
	this->hold_releaseCallback =hrcb;
	this->quickpressCallback =qpcb;
	initialise(pin_t);
}
Button::~Button (){
	delete(this->pin);
	delete(this->stateTimer);
}
void Button::stop() {
	this->stateTimer->cancel();
	this->state = suspend;
	this->event = None;

}

void Button::start() {
	event = None;
	neutralSA();
}

void Button::initialise(int pin_t){
	this->pin = new myGPIO(pin_t);
	this->pin->setDirection(myGPIO::INPUT);
	this->stateTimer = new StateTimer();
	this->pin->setActiveHigh();
	this->quickpress_duration = QUICKPRESS_DUR; //100ms
	// Initialize states and transition.
//	start();

}
//
//
void Button::onRisingEdge(){
	// always disable button timer first.

	this->stateTimer->cancel();
	// cout << "[BTN] risingedge event" <<endl;
	this->event = risingedge;
	this->transition();
}
void Button::onFallingEdge(){
	// always disable button timer first.
	this->stateTimer->cancel();
	// cout << "[BTN] fallingedge event" <<endl;
	this->event = fallingedge;
	this->transition();
}

void Button::onExceedThreshold(){
	// always disable button timer first.
	this->stateTimer->cancel();
	// cout << "[BTN] Thres Exceed Event" <<endl;

	this->event = exceedthres;
	this->transition();
}
// *****************State Actions **************
void Button::neutralSA() {
	this->state = neutral_0;

	this->pin->setEdgeType(myGPIO::RISING);
	this->pin->waitForEdge(boost::bind( &Button::onRisingEdge, this ), -1);
	// cout << "[BTN] neutral state" <<endl;
}


void Button::timingSA() {
	this->state = timing;

	// Execute actions for timing state.
	// attach a timer callback here.


	this->stateTimer->start(boost::bind( &Button::onExceedThreshold, this), this->quickpress_duration );

	this->pin->setEdgeType(myGPIO::FALLING);

	this->pin->waitForEdge(boost::bind( &Button::onFallingEdge, this ), this->quickpress_duration);
	// cout << "[BTN] timing state" <<endl;
}

void Button::holdSA() {

	this->state = hold;
	this->pin->setEdgeType(myGPIO::FALLING);
	this->pin->waitForEdge(boost::bind( &Button::onFallingEdge, this ), -1);
	if (!this->holdCallback.empty()) {
		(this->holdCallback)();
	}
	// cout << "[BTN] hold state" <<endl;
}

void Button::quickPressSA() {
	this->state = quickpress;
	// cout << "[BTN] quickpress state" <<endl;
	// do some trigger. here..
	if (!this->quickpressCallback.empty()) {
		(this->quickpressCallback)();
	}
	//Change to neutral state
	neutralSA();
}


void Button::transition() {
	//	// TODO: add time in neutral_0
	//	static int timing_timer;
	//	static int neutral_timer;
	//	int neutral_dur = -1;
	//
	switch (this->state){
	case neutral_0:
		if (this->event == risingedge) {
			this->state = timing;
			timingSA();
		}
		break;
	case timing:

		if (this->event == exceedthres){
			this->state = hold;
			holdSA();
		} else if (this->event == fallingedge) {
			this->state = quickpress;
			quickPressSA();
		}
		break;
	case hold:
		// Check for falling edge.
		if (this->event == fallingedge){
			this->state =neutral_0;
			neutralSA();
			// inject hold release callback here.
			if (!this->hold_releaseCallback.empty()) {
				(this->hold_releaseCallback)();
			}
		}
		break;
	case quickpress:
			break;
	default:
			break;
	}


	//
}
