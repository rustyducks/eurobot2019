/*
 * ExtNavigation.cpp
 *
 *  Created on: 12 déc. 2017
 *      Author: fabien
 */

#include <ExtNavigation.h>
#include "Arduino.h"
#include "Odometry.h"
#include "utilities.h"
#include "MotorControl.h"
#include "communication/Communication.h"
#include "InputOutputs.h"

ExtNavigation extNavigation = ExtNavigation();

ExtNavigation::ExtNavigation() {
	_omega_setpoint = 0;
	_speed_setpoint = 0;
}

ExtNavigation::~ExtNavigation() {
}

void ExtNavigation::update() {

	float speed_setpoint;
	float omega_setpoint;

	if(fat::communication.getTimeSinceLastSpeedMessage() < TIME_SPEED_FAILSAFE) {
		speed_setpoint = _speed_setpoint;
		omega_setpoint = _omega_setpoint;
	} else {
		speed_setpoint = 0;
		omega_setpoint = 0;
		inputOutputs.HMISetLedColor(255,0,0);
		Serial.print("[WARNING] Long time since last speed message. Stop for failsafe. ");
	}
	motorControl.set_speed_setpoint(speed_setpoint);
	motorControl.set_omega_setpoint(omega_setpoint);
}

void ExtNavigation::reset() {
	_omega_setpoint = 0;
	_speed_setpoint = 0;
}
