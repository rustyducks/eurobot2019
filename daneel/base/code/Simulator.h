/*
 * Simulator.h
 *
 *  Created on: 20 déc. 2017
 *      Author: fabien
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "params.h"

class Simulator {
public:
	Simulator();
	virtual ~Simulator();

	int readEnc(int moteurNb);
	void setMotorCommand(int command, int motor);
	void update();

private:
	float vDiff[3];
	float acc[3];

	float vFinal[3];
	float v[3];

	float thau;

	float encs[3];
	float KMotor;
};

#ifdef SIMULATOR
extern Simulator simulator;
#endif

#endif /* SIMULATOR_H_ */
