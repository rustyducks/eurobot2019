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
	_theta = 0;
	_inc1 = _inc2 = _inc3 = 0;
	_speed = makeSpeed(0,0,0);
	_motorSpeeds = makeSpeed(0,0,0);
	_motorsDisplacement = makeSpeed(0,0,0);
	_robotDisplacement = makeSpeed(0,0,0);
	_moveDelta = makeSpeed(0,0,0);				//TODO : do makeMove I know what I am doing. Don't do that if you don't have 3 motors
}

Odometry::~Odometry() {
}

float Odometry::getDeltaTheta() {
	return _moveDelta->pData[3];
}

void Odometry::update() {
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
}

void Odometry::updateDifferential() {
	//TODO (or not)
}

void Odometry::updateHolonomic() {
	cli();				//disable interrupts to copy and reset the counters
	int inc1 = _inc1;
	int inc2 = _inc2;
	int inc3 = _inc3;
	_inc1 = 0;
	_inc2 = 0;
	_inc3 = 0;
	sei();				//enable interrupts

#ifdef SIMULATOR
	inc1 = simulator.readEnc(0);
	inc2 = simulator.readEnc(1);
	inc3 = simulator.readEnc(2);
#endif

	//get motor displacement
	_motorsDisplacement->pData[0] = (float32_t)inc1 / INC_PER_MM;
	_motorsDisplacement->pData[1] = (float32_t)inc2 / INC_PER_MM;
	_motorsDisplacement->pData[2] = (float32_t)inc3 / INC_PER_MM;

	//compute robot displacement, according to robot fame
	arm_status status = arm_mat_mult_f32(&Dplus, _motorsDisplacement, _robotDisplacement);
	if(status != ARM_MATH_SUCCESS) {
		Serial.print("[ERROR] updateHolonomic: matrix multiplication error : ");
		Serial.println(status);
	}

	//update robot position in table frame
	updatePosition();


	//get motors speed.
	//For now, useful for the motors control.
	_motorSpeeds->pData[0] = _motorsDisplacement->pData[0] / CONTROL_PERIOD;
	_motorSpeeds->pData[1] = _motorsDisplacement->pData[1] / CONTROL_PERIOD;
	_motorSpeeds->pData[2] = _motorsDisplacement->pData[2] / CONTROL_PERIOD;

	//get robot speed
	//should give the same result than :
	//arm_status status = arm_mat_mult_f32(&Dplus, _motorSpeeds, _speed);
	_speed->pData[0] = _robotDisplacement->pData[0] / CONTROL_PERIOD;
	_speed->pData[1] = _robotDisplacement->pData[1] / CONTROL_PERIOD;
	_speed->pData[2] = _robotDisplacement->pData[2] / CONTROL_PERIOD;	//TODO RW_to_W ?
}

void Odometry::recalerTheta(float32_t theta) {
	_theta = theta + _moveDelta->pData[2];
}

void Odometry::updatePosition() {
	float32_t dTheta = RW_to_W(_robotDisplacement->pData[2]);		//transforms speed in dTheta
	_theta += dTheta;		//update _theta with the last dTheta.

	//change speed reference system from robot reference frame to table reference frame
	//TODO : do it with matrix multiplication
	float32_t dx = _robotDisplacement->pData[0] * cos(_theta) - _robotDisplacement->pData[1] * sin(_theta);
	float32_t dy = _robotDisplacement->pData[1] * cos(_theta) + _robotDisplacement->pData[0] * sin(_theta);

	//TODO use function from arm_math ?
	//Add this move to the other (table reference system)
	_moveDelta->pData[0]+= dx;
	_moveDelta->pData[1]+= dy;
	_moveDelta->pData[2]+= dTheta;

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
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	pinMode(MOT2_ENCA, INPUT);
	pinMode(MOT2_ENCB, INPUT);
	pinMode(MOT3_ENCA, INPUT);
	pinMode(MOT3_ENCB, INPUT);
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

void Odometry::resetMoveDelta() {
	_moveDelta->pData[0] = 0;
	_moveDelta->pData[1] = 0;
	_moveDelta->pData[2] = 0;
}

void Odometry::reset() {
	cli();
	_theta = 0;
	_inc1 = _inc2 = _inc3 = 0;
	zeroMatrix(_moveDelta);
	zeroMatrix(_motorsDisplacement);
	zeroMatrix(_robotDisplacement);
	zeroMatrix(_motorSpeeds);
	zeroMatrix(_speed);
	sei();
}
