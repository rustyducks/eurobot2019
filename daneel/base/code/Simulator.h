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
	void analogWrite(int motor, int value);
	void digitalWrite(int motor, int value);
	void update();

private:
	float vDiff[3];
	float acc[3];

	float vFinal[3];
	float v[3];

	float thau;

	int signs[3];

	float encs[3];
	float KMotor;
};

#ifdef SIMULATOR
extern Simulator simulator;
#endif

#endif /* SIMULATOR_H_ */
