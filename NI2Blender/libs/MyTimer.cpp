/*
 * MyTimer.cpp
 *
 *  Created on: Feb 9, 2012
 *      Author: fabio
 */

#include <time.h>
#include "MyTimer.h"
#include <iostream>

MyTimer::MyTimer() {
	resetted = true;
	running = false;
	beg = 0;
	end = 0;
}

void MyTimer::start() {
//	if (!running) {
		if (resetted)
			beg = (unsigned long) clock();
		else
			beg -= end - (unsigned long) clock();
		running = true;
		resetted = false;
//	}
}

void MyTimer::stop() {
//	if (running) {
		end = (unsigned long) clock();
		running = false;
//	}
}

void MyTimer::reset() {
//	bool wereRunning = running;
//	if (wereRunning)
		stop();
	resetted = true;
	beg = 0;
	end = 0;
//	if (wereRunning)
//		start();
}

bool MyTimer::isRunning() {
	return running;
}

unsigned long MyTimer::getTime() {
	if (running)
		return ((unsigned long) clock() - beg);// / CLOCKS_PER_SEC;
	else
		return end - beg;// / CLOCKS_PER_SEC;
}

bool MyTimer::isOver(unsigned long clocks) {
	//unsigned long t = getTime();
	/*if(!(clocks >= t)){
		//std::cout << "\nTime >" << t;
		stop();
		reset();
	}*/
	return clocks >= getTime();
}
