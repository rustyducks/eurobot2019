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

#ifdef SIMULATOR
#include "Simulator.h"
#endif

#define sig(x) (x) > 0 ? 0 : 1

MotorControl motorControl = MotorControl();

MotorControl::MotorControl() {
	_intError_speed = _prevError_speed = _intError_omega = _prevError_omega = 0;
	_speed_setpoint = _omega_setpoint = 0;

	KP_SPEED = 0.6;
	KI_SPEED = 0.2;
	KD_SPEED = 0;
	KP_OMEGA = 90;
	KI_OMEGA = 7;
	KD_OMEGA = 0;
}

void MotorControl::init() {
	analogWriteResolution(8);

	pinMode(MOT1_PWM, OUTPUT);
	pinMode(MOT1_DIR, OUTPUT);
	analogWriteFrequency(MOT1_PWM, PWM_FREQUENCY);

	pinMode(MOT2_PWM, OUTPUT);
	pinMode(MOT2_DIR, OUTPUT);
	analogWriteFrequency(MOT2_PWM, PWM_FREQUENCY);
}

void MotorControl::control() {

	float error_speed = _speed_setpoint - odometry.get_speed();
	_intError_speed += error_speed;
	float delta_speed_error = error_speed - _prevError_speed;
	_prevError_speed = error_speed;
	float cmd_speed = KP_SPEED * error_speed + KI_SPEED * _intError_speed - KD_SPEED * delta_speed_error;


	float error_omega = _omega_setpoint - odometry.get_omega();
	_intError_omega += error_omega;
	_intError_omega = clamp((float)-50, _intError_omega + error_omega, (float)50);
	float delta_omega_error = error_omega - _prevError_omega;
	_prevError_omega = error_omega;
	float cmd_omega = KP_OMEGA * error_omega + KI_OMEGA * _intError_omega + KD_OMEGA * delta_omega_error;

	int cmd_mot1 = clamp(-PWM_MAX, (int)(cmd_speed - cmd_omega), PWM_MAX);
	int cmd_mot2 = -clamp(-PWM_MAX, (int)(cmd_speed + cmd_omega), PWM_MAX);

	setMotorCommand(cmd_mot1, MOT1_PWM, MOT1_DIR);
	setMotorCommand(cmd_mot2, MOT2_PWM, MOT2_DIR);

#ifdef SIMULATOR
	simulator.setMotorCommand(cmd_mot1, 0);
	simulator.setMotorCommand(cmd_mot2, 1);
#endif
}

void MotorControl::setMotorCommand(int command, int pwmPin, int dirPin) {
	digitalWrite(dirPin, sig(command));
	analogWrite(pwmPin, abs(command));
}

void MotorControl::reset() {
	setMotorCommand(0, MOT1_PWM, MOT1_DIR);
	setMotorCommand(0, MOT2_PWM, MOT2_DIR);

	_intError_speed = _prevError_speed = _intError_omega = _prevError_omega = 0;
	_speed_setpoint = _omega_setpoint = 0;
}
