/*
 * BallDetector.cpp
 *
 *  Created on: 4 mai 2018
 *      Author: guilhem
 */

#include <BallDetector.h>
#include <Arduino.h>

BallDetector::BallDetector(uint8_t readPin):ballDetectedThreshold(0), noBallDetectedThreshold(0),
		state(eState::NO_BALL), ballNumber(0), readPin(readPin){

	pinMode(readPin, INPUT);
}

void BallDetector::detect(){
//	Serial.print("Analog : ");
//	Serial.println(analogRead(readPin));
	switch(state){
	case eState::NO_BALL:
		if (analogRead(readPin) >= ballDetectedThreshold){
			state = eState::BALL;
		}
		break;
	case eState::BALL:
		if (analogRead(readPin) <= noBallDetectedThreshold){
			state = eState::NO_BALL;
			ballNumber++;
		}
		break;
	}
}

unsigned int BallDetector::getNumberOfBalls(){
	return ballNumber;
}

void BallDetector::setThresholds(int noBallThreshold, int ballThreshold){
	noBallDetectedThreshold = noBallThreshold;
	ballDetectedThreshold = ballThreshold;
}

void BallDetector::reset(){
	state = eState::NO_BALL;
	ballNumber = 0;
}

BallDetector::~BallDetector() {
	// TODO Auto-generated destructor stub
}

