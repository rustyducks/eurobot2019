/*
 * BaseHolonomy.cpp
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#include <Arduino.h>
#include <Odometry.h>
#include "params.h"
#include "utilities.h"

#ifdef SIMULATOR
#include "Simulator.h"
#endif

Odometry odometry = Odometry();

Odometry::Odometry() {
	_x = _y = _theta = 0;
	_speed = _omega = 0;
	_incr1 = _incr2 = 0;
}

Odometry::~Odometry() {
}


void Odometry::update() {
	cli();
	int incr1 = _incr1;
	int incr2 = _incr2;
	_incr1 = _incr2 = 0;
	sei();

#ifdef SIMULATOR
	incr1 = simulator.readEnc(0);
	incr2 = simulator.readEnc(1);
#endif

	float length = ((float)(-incr1+incr2)/2.0)/INC_PER_MM;		//opposite sign on incr1 because motors are mirrored
	float angle = ((float)(incr1+incr2)/INC_PER_MM)/WHEELBASE;  //opposite sign on incr1 because motors are mirrored

	_x = _x + length*cos(_theta + angle/2.0);
	_y = _y + length*sin(_theta + angle/2.0);
	_theta = _theta + angle;
	_speed = length / CONTROL_PERIOD;
	_omega = angle / CONTROL_PERIOD;
}

void Odometry::init() {
	reset();
}


void initOdometry() {
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	pinMode(MOT2_ENCA, INPUT);
	pinMode(MOT2_ENCB, INPUT);
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
	attachInterrupt(MOT2_ENCA, ISR2, RISING);
	attachInterrupt(MOT2_ENCB, ISR22, RISING);

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

void Odometry::isr1() {
	if(digitalRead(MOT1_ENCB)) {
		_incr1++;
	} else {
		_incr1--;
	}
}

void Odometry::isr11() {
	if(digitalRead(MOT1_ENCA)) {
		_incr1--;
	} else {
		_incr1++;
	}
}

void Odometry::isr2() {
	if(digitalRead(MOT2_ENCB)) {
		_incr2++;
	} else {
		_incr2--;
	}
}

void Odometry::isr22() {
	if(digitalRead(MOT2_ENCA)) {
		_incr2--;
	} else {
		_incr2++;
	}
}

void Odometry::reset() {
	cli();
	_x = _y = _theta = 0;
	_speed = _omega = 0;
	_incr1 = _incr2 = 0;
	sei();
}
