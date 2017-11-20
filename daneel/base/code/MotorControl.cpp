/*
 * BaseMotorControl.cpp
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#include <Arduino.h>
#include <MotorControl.h>
#include "params.h"
#include "utilities.h"
#include <DynamixelSerial5.h>

MotorControl motorControl = MotorControl();

MotorControl::MotorControl() {
	_intSpeedError = 0;
	_intHeadingError = 0;

}

void MotorControl::init() {

	analogWriteResolution(PWM_RESOLUTION);

#if defined(TRIKE)
	pinMode(MOT1_PWM, OUTPUT);
	pinMode(MOT1_DIR, OUTPUT);
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	analogWriteFrequency(MOT1_PWM, PWM_FREQUENCY);

#elif defined(DIFFERENTIAL)
	pinMode(MOT1_PWM, OUTPUT);
	pinMode(MOT1_DIR, OUTPUT);
	pinMode(MOT1_BRK, OUTPUT);
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	analogWriteFrequency(MOT1_PWM, PWM_FREQUENCY);

	pinMode(MOT2_PWM, OUTPUT);
	pinMode(MOT2_DIR, OUTPUT);
	pinMode(MOT2_BRK, OUTPUT);
	pinMode(MOT2_ENCA, INPUT);
	pinMode(MOT2_ENCB, INPUT);
	analogWriteFrequency(MOT2_PWM, PWM_FREQUENCY);

#elif defined(HOLONOMIC)
	pinMode(MOT1_PWM, OUTPUT);
	pinMode(MOT1_DIR, OUTPUT);
	pinMode(MOT1_BRK, OUTPUT);
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	analogWriteFrequency(MOT1_PWM, PWM_FREQUENCY);

	pinMode(MOT2_PWM, OUTPUT);
	pinMode(MOT2_DIR, OUTPUT);
	pinMode(MOT2_BRK, OUTPUT);
	pinMode(MOT2_ENCA, INPUT);
	pinMode(MOT2_ENCB, INPUT);
	analogWriteFrequency(MOT2_PWM, PWM_FREQUENCY);

	pinMode(MOT3_PWM, OUTPUT);
	pinMode(MOT3_DIR, OUTPUT);
	pinMode(MOT3_BRK, OUTPUT);
	pinMode(MOT3_ENCA, INPUT);
	pinMode(MOT3_ENCB, INPUT);
	analogWriteFrequency(MOT3_PWM, PWM_FREQUENCY);
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif
}

void MotorControl::control() {
#if defined(TRIKE)
	controlTrike();
#elif defined(DIFFERENTIAL)
	controlDifferential();
#elif defined(HOLONOMIC)
	controlHolonomic();
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif

}

void MotorControl::controlTrike() {
	float speedNorm = sqrtf(powf(odometry.getSpeed().getVx(), 2) + powf(odometry.getSpeed().getVy(), 2));
	//targetSpeedNorm is the speed of the wheel, NOT the robot !
	float targetSpeedNorm = sqrtf(powf(_targetSpeed.getVx(), 2) + powf(_targetSpeed.getVy(), 2));
	float previousSpeedNorm = sqrtf(powf(odometry.getPreviousSpeed().getVx(), 2) + powf(odometry.getPreviousSpeed().getVy(), 2));

	float speedError = targetSpeedNorm - speedNorm;
	_intSpeedError += speedError;

	int motorCmd = KP * speedError + KI * _intSpeedError - KD * (speedNorm - previousSpeedNorm);
	if(motorCmd > 0) {
		digitalWrite(MOT1_DIR, HIGH);
	} else {
		digitalWrite(MOT1_DIR, LOW);
	}
	motorCmd = clamp(0, abs(motorCmd), PWM_MAX);
	analogWrite(MOT1_PWM, motorCmd);

	float sinAlpha = (WHEELBASE * _targetSpeed.getOmega()) / targetSpeedNorm;
	float dyna_angle = asin(clamp(-1.f, sinAlpha, 1.f));
	int dyna_command = dyna_angle * RAD_TO_DYNA + DYNA_ZERO;
	dyna_command = clamp(0, dyna_command, 1023);
	Dynamixel.move(STEER_DYNA_ID, dyna_command);
}

void MotorControl::controlDifferential() {
}

void MotorControl::controlHolonomic() {
	//TODO converter the error speed in the robot reference system
	Speed3D error = _targetSpeed - odometry.getSpeed();
}
