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
	unsigned int getNumberOfBalls();
	virtual ~BallDetector();
private:
	static constexpr int BALL_DETECTED_THRESHOLD = 500;
	static constexpr int NO_BALL_DETECTED_THRESHOLD = 200;

	enum eState{
		NO_BALL,
		BALL
	};
	eState state;
	unsigned int ballNumber;
	uint8_t readPin;
};

#endif /* LIBRARIES_BALLDETECTOR_BALLDETECTOR_H_ */
