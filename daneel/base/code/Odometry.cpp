/*
 * BaseHolonomy.cpp
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#include <Arduino.h>
#include <Odometry.h>
#include "params.h"
#include <DynamixelSerial5.h>


Odometry odometry = Odometry();

Odometry::Odometry() {
	_readIndex = 0;
	_thetaAI = 0;
	_inc1 = _inc2 = _inc3 = 0;
}

Odometry::~Odometry() {
}

Move3D Odometry::getMoveDelta(int originId) {
	Move3D move;
	int currentIndex = _readIndex;

	//TODO correct potential bug : if originId < lastId - MOVE_HISTORY_LENGHT, it will loop forever !
	//Add a counter or whatever to protect against this bug.

	//sum the moves from id originId to the most recent (in reverse order because it's easier)
	do {
		move = move + _moveDelta[currentIndex];		//TODO : use += operator
		currentIndex--;
	} while(_moveDelta[currentIndex].getId()>originId);

	return move;
}

float Odometry::getDeltaTheta() {
	return _moveDelta[_readIndex].getTheta();
}

void Odometry::update() {
	//Don't know the behavior of `_previousSpeed = _speed`
	_previousSpeed.setVx(_speed.getVx());
	_previousSpeed.setVy(_speed.getVy());
	_previousSpeed.setOmega(_speed.getOmega());

#if defined(TRIKE)
	updateTrike();
#elif defined(DIFFERENTIAL)
	updateDifferential();
#elif defined(HOLONOMIC)
	updateHolonomic();
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif
}

void Odometry::init() {
}

void Odometry::updateTrike() {
	float angle = Dynamixel.readPosition(STEER_DYNA_ID);	//TODO convert dynamixel angle in radians

	int inc1 = _inc1;	//work on copy : interrupts are dangerous !
	_inc1 = 0;			//reset the increment number

	float length = float(inc1) * WHEEL_PERIMETER / INCREMENTS_PER_ROTATION;

	float rotation;
	if(angle > MINIMUM_ANGLE) {
		float radius = WHEELBASE / tan(angle);	//radius of the circle made by the robot
		rotation = length / radius;
	}
	else {
		rotation = 0;
	}

	Move3D move = Move3D(length, 0, rotation);	//TODO do better estimation (refine x and y)
	addMove(move);

	Speed3D speed = Speed3D(length/CONTROL_PERIOD, 0, rotation / CONTROL_PERIOD);
	setSpeed(speed);

}

void Odometry::updateDifferential() {
	//TODO (or not)
}

void Odometry::updateHolonomic() {

}

void Odometry::setSpeed(const Speed3D& speed) {
	float theta = _thetaAI + getDeltaTheta();
	float vx = speed.getVx() * cos(theta) - speed.getVy() * sin(theta);
	float vy = speed.getVy() * cos(theta) + speed.getVx() * sin(theta);
	_speed.setVx(vx);
	_speed.setVy(vy);
}

void Odometry::addMove(Move3D move) {
	float theta = _thetaAI + getDeltaTheta();

	//change reference system
	float x = move.getX() * cos(theta) - move.getY() * sin(theta);
	float y = move.getY() * cos(theta) + move.getX() * sin(theta);

	//modify the object rather than create a new one
	move.setX(x);
	move.setY(y);

	//Add this move to the other
	_moveDelta[_readIndex] = _moveDelta[_readIndex] + move;	//TODO Use += operator
}

void initOdometry() {
#if defined(TRIKE)
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
#elif defined(DIFFERENTIAL)
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
	attachInterrupt(MOT2_ENCA, ISR2, RISING);
	attachInterrupt(MOT2_ENCB, ISR22, RISING);
#elif defined(HOLONOMIC)
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
	attachInterrupt(MOT2_ENCA, ISR2, RISING);
	attachInterrupt(MOT2_ENCB, ISR22, RISING);
	attachInterrupt(MOT3_ENCA, ISR3, RISING);
	attachInterrupt(MOT3_ENCB, ISR33, RISING);
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif

	odometry.init();
}

void ISR1() {
	odometry.isr1();
}

void ISR11() {
	odometry.isr11();
}

void ISR2() {
	odometry.isr2();
}

void ISR22() {
	odometry.isr22();
}

void ISR3() {
	odometry.isr3();
}

void ISR33() {
	odometry.isr33();
}

void Odometry::isr1() {
	if(digitalRead(MOT1_ENCB)) {
		_inc1++;
	} else {
		_inc1--;
	}
}

void Odometry::isr11() {
	if(digitalRead(MOT1_ENCA)) {
		_inc1--;
	} else {
		_inc1++;
	}
}

void Odometry::isr2() {
	if(digitalRead(MOT2_ENCB)) {
		_inc2++;
	} else {
		_inc2--;
	}
}

void Odometry::isr22() {
	if(digitalRead(MOT2_ENCA)) {
		_inc2--;
	} else {
		_inc2++;
	}
}

void Odometry::isr3() {
	if(digitalRead(MOT3_ENCB)) {
		_inc3++;
	} else {
		_inc3--;
	}
}

void Odometry::isr33() {
	if(digitalRead(MOT3_ENCA)) {
		_inc3--;
	} else {
		_inc3++;
	}
}
