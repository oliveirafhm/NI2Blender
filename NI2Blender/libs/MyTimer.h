/*
 * MyTimer.h
 *
 *  Created on: Feb 9, 2012
 *      Author: fabio
 */

#ifndef MYTIMER_H_
#define MYTIMER_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h> // Only POSIX =/
using namespace std;

class Timer {
private:

	timeval startTime;
	timeval endTime;
	bool running;

public:

	Timer() {
		running = false;
	}

	void start() {
		if (!running) {
			gettimeofday(&startTime, NULL);
			running = true;
		}
	}

	void stop(bool changeRunning = true) {
		if (running) {
			gettimeofday(&endTime, NULL);
			if(changeRunning)running = false;
		}
	}

	void reset() {
		stop();
		start();
	}

	bool isRunning() {
		return running;
	}
	// In seconds
	double elapsedTime() {
		if (running) stop(false);

		long seconds, useconds;
		double duration;

		seconds = endTime.tv_sec - startTime.tv_sec;
		useconds = endTime.tv_usec - startTime.tv_usec;

		duration = seconds + useconds / 1000000.0;
		//duration = ((seconds) * 1000 + useconds / 1000.0) + 0.5;//For milliseconds

		return duration;
	}

	bool isOver(double seconds) {
		//printf("\nElapsed Time: %f", elapsedTime());
		return seconds > elapsedTime();
	}

	static void printTime(double duration) {
		printf("%5.3f seconds\n", duration);
	}
};

#endif /* MYTIMER_H_ */
