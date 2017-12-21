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
	KMotor = 1;
	thau = 400;
	vDiff[0] = vDiff[1] = vDiff[2] = 0;
	vFinal[0] = vFinal[1] = vFinal[2] = 0;
	v[0] = v[1] = v[2] = 0;
	encs[0] = encs[1] = encs[2] = 0;
}

Simulator::~Simulator() {
	// TODO Auto-generated destructor stub
}

int Simulator::readEnc(int motorNb) {
	int tmp = encs[motorNb];
	encs[motorNb] = 0;
	return tmp;
}

void Simulator::update() {
	for(int i; i<3;i++) {
		vDiff[i] = vDiff[i] + acc[i];
		acc[i] = -vDiff[i]/thau;
		v[i] = vDiff[i] + vFinal[i];
		encs[i] += v[i] * KMotor;
	}
}

void Simulator::setMotorCommand(int command, int motor) {
	vFinal[motor] = command;
	vDiff[motor] = v[motor] - vFinal[motor];
}
