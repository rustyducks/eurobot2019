/*
 * Simulator.cpp
 *
 *  Created on: 20 déc. 2017
 *      Author: fabien
 */

#include <Simulator.h>
#include <Arduino.h>

#ifdef SIMULATOR
Simulator simulator = Simulator();
#endif

Simulator::Simulator() {
	KMotor = 2.5;
	tau = 0.2;
	vDiff[0] = vDiff[1] = vDiff[2] = 0;
	vFinal[0] = vFinal[1] = vFinal[2] = 0;
	v[0] = v[1] = v[2] = 0;
	encs[0] = encs[1] = encs[2] = 0;
}

Simulator::~Simulator() {
	// TODO Auto-generated destructor stub
}

int Simulator::readEnc(int motorNb) {
	float tmp = encs[motorNb];
	encs[motorNb] = 0;
	return (int) tmp;
}

void Simulator::update() {
	for(int i=0; i<3;i++) {
		vDiff[i] = vDiff[i] + acc[i] * CONTROL_PERIOD;
		acc[i] = -vDiff[i]/tau;
		v[i] = vDiff[i] + vFinal[i];
		encs[i] += v[i] * CONTROL_PERIOD * INC_PER_MM;

	}
}

void Simulator::setMotorCommand(int command, int motor) {
	vFinal[motor] = command * KMotor;
	vDiff[motor] = v[motor] - vFinal[motor];
}

void Simulator::reset() {
	for(int i=0; i<3; i++) {
		acc[i] = 0;
		vFinal[i] = 0;
		v[i] = 0;
		encs[i] = 0;
	}
}
