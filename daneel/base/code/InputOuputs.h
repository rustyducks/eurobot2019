/*
 * InputOuputs.h
 *
 *  Created on: 22 déc. 2017
 *      Author: fabien
 */

#ifndef INPUTOUPUTS_H_
#define INPUTOUPUTS_H_

class InputOuputs {
public:
	InputOuputs();
	virtual ~InputOuputs();

	void init();

	void HMISetLedColor(int red, int green, int blue);

	int HMIGetColorState();

	int HMIGetTiretteState();

private:



};

#endif /* INPUTOUPUTS_H_ */
