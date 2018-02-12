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

	void handleActuatorMessage(int actuatorId, int actuatorCommand);
	void deliverWater(bool enable);

	void _onNewCordState();
	void _onNewButtonState(int button);

	bool isHmIhasChanged() const {
		return _HMIhasChanged;
	}

	void setHmIhasChanged(bool hmIhasChanged) {
		_HMIhasChanged = hmIhasChanged;
	}

private:
	typedef enum{
		WATER_DELIVERING_DYNAMIXEL = 0,
		WATER_CANNON_DC_MOTOR = 1
	}eMsgActuatorId;
	bool _button1Pressed;
	bool _button2Pressed;
	bool _cordIn;
	bool _redLEDOn;
	bool _greenLEDOn;
	bool _blueLEDOn;
	
	volatile bool _HMIhasChanged;
};

extern InputOutputs inputOutputs;

#endif /* INPUTOUTPUTS_H_ */
