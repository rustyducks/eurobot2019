/*
 * BallDetector.h
 *
 *  Created on: 4 mai 2018
 *      Author: guilhem
 */

#ifndef LIBRARIES_BALLDETECTOR_BALLDETECTOR_H_
#define LIBRARIES_BALLDETECTOR_BALLDETECTOR_H_

#include <stdint.h>

class BallDetector {
public:
	BallDetector(uint8_t readPin);
	void detect();
	void reset();
	void setThresholds(int noBallThreshold, int ballThreshold);
	unsigned int getNumberOfBalls();
	virtual ~BallDetector();
private:
	enum eState{
		NO_BALL,
		BALL
	};
	int ballDetectedThreshold;
	int noBallDetectedThreshold;
	eState state;
	unsigned int ballNumber;
	uint8_t readPin;
};

#endif /* LIBRARIES_BALLDETECTOR_BALLDETECTOR_H_ */
