/*
 * InputOuputs.h
 *
 *  Created on: 22 déc. 2017
 *      Author: fabien
 */

#ifndef INPUTOUTPUTS_H_
#define INPUTOUTPUTS_H_

class InputOutputs {
public:
	InputOutputs();
	virtual ~InputOutputs();

	void init();

	void HMISetLedColor(int red, int green, int blue);

	bool HMIGetButton1State();
	bool HMIGetButton2State();

	bool HMIGetCordState();
	void HMISendState();

	void _onNewCordState();
	void _onNewButtonState(int button);

private:
	bool button1Pressed;
	bool button2Pressed;
	bool cordIn;
	bool redLEDOn;
	bool greenLEDOn;
	bool blueLEDOn;
};

extern InputOutputs inputOutputs;

#endif /* INPUTOUTPUTS_H_ */
