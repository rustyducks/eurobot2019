/*
 * BallDetector.cpp
 *
 *  Created on: 4 mai 2018
 *      Author: guilhem
 */

#include <BallDetector.h>
#include <Arduino.h>

BallDetector::BallDetector(uint8_t readPin): state(eState::NO_BALL), ballNumber(0),
		readPin(readPin){

	pinMode(readPin, INPUT);
}

void BallDetector::detect(){
//	Serial.print("Analog : ");
//	Serial.println(analogRead(readPin));
	switch(state){
	case eState::NO_BALL:
		if (analogRead(readPin) >= BALL_DETECTED_THRESHOLD){
			state = eState::BALL;
		}
		break;
	case eState::BALL:
		if (analogRead(readPin) <= NO_BALL_DETECTED_THRESHOLD){
			state = eState::NO_BALL;
			ballNumber++;
		}
		break;
	}
}

unsigned int BallDetector::getNumberOfBalls(){
//	Serial.print("Balls : ");
//	Serial.println(ballNumber);
	return ballNumber;
}

void BallDetector::reset(){
	state = eState::NO_BALL;
	ballNumber = 0;
}

BallDetector::~BallDetector() {
	// TODO Auto-generated destructor stub
}

