#include "controller_input/gpio.h"
using namespace std;

myGPIO::myGPIO(int number) :
		GPIO(number) {
	pthread_attr_init(&tattr);
	pthread_attr_setdetachstate(&this->tattr, PTHREAD_CREATE_DETACHED);
	epollfd = epoll_create(1);
}
int myGPIO::waitForEdge(boost::function<void(void)> callback, int duration) {
	 m.lock();
	this->duration = duration;
	this->threadRunning = true;
	this->callback = callback;
	// create the thread, pass the reference, address of the function and data
	int ret =0;
	/* initialized with default attributes */

	if (pthread_create(&this->thread, &this->tattr, &mythreadedPoll,
			static_cast<void*>(this))) {
		perror("GPIO: Failed to create the poll thread");
		this->threadRunning = false;
		ret = -1;
	}
	 m.unlock();

	return ret;

}

// This thread function is a friend function of the class

void* mythreadedPoll(void *value) {

	myGPIO *gpio = static_cast<myGPIO*>(value);
	if ((gpio->_waitForEdge(gpio->duration) > 0) && gpio->threadRunning) {
		((gpio->callback))();
	}
	usleep(gpio->debounceTime * 1000);

	return 0;
}

int myGPIO::_waitForEdge(int duration) {
	this->setDirection(INPUT); // must be an input pin to poll its value
	int fd, i, count=0;
	struct epoll_event ev;

    if (epollfd == -1) {
	   perror("GPIO: Failed to create epollfd");
	   return -1;
    }
    if ((fd = open((this->path + "value").c_str(), O_RDONLY | O_NONBLOCK)) == -1) {
       perror("GPIO: Failed to open file");
       return -1;
    }

    //ev.events = read operation | edge triggered | urgent data
    ev.events = EPOLLIN | EPOLLET | EPOLLPRI;
    ev.data.fd = fd;  // attach the file file descriptor

    //Register the file descriptor on the epoll instance, see: man epoll_ctl

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev) == -1) {
       perror("GPIO: Failed to add control interface");
       return -1;
    }
//    // cout << "[myGPIO] pin << "<<this->number<<"; fd no: " << fd << endl;

	while(count<=1){  // ignore the first trigger
//		i = epoll_wait(epollfd, &ev,1, duration);
		do {
		    i = epoll_wait(epollfd, &ev, 1, duration);
		} while (i < 0 && errno == EINTR);

		if (i==-1){
			perror("GPIO: Poll Wait fail");
			count=5; // terminate loop
		}
		else {
			count++; // count the triggers up
		}
	}
    close(fd);
    if (count==5) return -1;
//    // cout << "[myGPIO] i = " << i << endl;
	return i;
}
///****************/
//	this->setDirection(INPUT); // must be an input pin to poll its value
//
//	int i, count = 0;
//
//	if (this->epollfd == -1) {
//		perror("GPIO: Failed to create epollfd");
//	}
//
//
//
////	//ev.events = read operation | edge triggered | urgent data
////	ev.events = EPOLLIN | EPOLLET | EPOLLPRI;
//
//
//	// Clear previous file monitor.
//	if (this->fd != -1) {
//		epoll_ctl(this->epollfd, EPOLL_CTL_DEL, this->fd, &ev);
//	}
//
//	if ((this->fd = open((this->path + "value").c_str(), O_RDONLY | O_NONBLOCK))
//			== -1) {
//		perror("GPIO: Failed to open file");
//		return -1;
//	}
//	// cout << "[myGPIO] fd no: " << this->fd << endl;
//
//	this->ev.data.fd = this->fd;  // attach the file file descriptor
//	int ret;
//	//Register the file descriptor on the epoll instance, see: man epoll_ctl
//	if ((ret = epoll_ctl(this->epollfd, EPOLL_CTL_ADD, this->fd, &ev)) == -1) {
//		perror("GPIO: Failed to add control interface");
//		return -1;
//	}
//	while (count <= 1) {  // ignore the first trigger
//		// Patch to check if wait for edge request is valid
////		if (((getValue() == HIGH) && (getEdgeType() == RISING))
////				|| ((getValue() == LOW) && (getEdgeType() == FALLING))) {
////			epoll_ctl(epollfd, EPOLL_CTL_DEL, this->fd, &ev);
////			close(this->fd);
////			return 5;
////		}
//
//		i = epoll_wait(this->epollfd, &this->ev, 1, duration);
//		// consume data available.
//		getValue();
//		if (i == -1) {
//			perror("GPIO: Poll Wait fail");
//			count = 5; // terminate loop
//		} else {
//			count++; // count the triggers up
//		}
//	}
//	close(this->fd);
//	if (count == 5)
//		return -1;
//	return i;
//}
