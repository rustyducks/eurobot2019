/*
 * InputOuputs.h
 *
 *  Created on: 22 déc. 2017
 *      Author: fabien
 */

#ifndef INPUTOUTPUTS_H_
#define INPUTOUTPUTS_H_
#include <cstdint>


#include <libraries/TM1637/TM1637Display.h>
#include <Servo.h>
#include <params.h>

class InputOutputs {
public:
	InputOutputs();
	virtual ~InputOutputs();

	void init();

	void run();

	void reset();

	void HMISetLedColor(int red, int green, int blue);

	bool HMIGetButton1State();
	bool HMIGetButton2State();

	bool HMIGetCordState();
	void HMISendState();

	void handleActuatorMessage(int actuatorId, int actuatorCommand);

	void handleSensorCommand(int sensorId, int sensorCommand);

	void _onNewCordState();
	void _onNewButtonState(int button);

	bool isHmIhasChanged() const {
		return _HMIhasChanged;
	}

	void setHmIhasChanged(bool hmIhasChanged) {
		_HMIhasChanged = hmIhasChanged;
	}


private:
	static constexpr int maxSensorNumber = 20;
	static constexpr int sensorPeriodicTime = 100; // ms
	typedef enum{
		ACT_VL6180X_LEFT_RESET = 0,
		ACT_VL6180X_CENTER_RESET = 1,
		ACT_VL6180X_RIGHT_RESET = 2,
		ACT_LIDAR_PWM = 3,
		ACT_SCORE_COUNTER = 4,
		ACT_EXPERIMENT_LAUCHER = 5

	}eMsgActuatorId;

	typedef struct{
		enum eSensorType{
			DIGITAL,
			ANALOG,
			DYNAMIXEL_POSITION,
		};
		enum eSensorId{
			BATTERY_SIG = 0,
			BATTERY_POW = 1,
		};
		typedef enum{
			STOPPED,
			ON_CHANGE,
			PERIODIC
		}eSensorReadState;
		eSensorType sensorType;
		eSensorId sensorId;
		eSensorReadState sensorReadState;
		long lastReadTime;
		int lastReadValue;
		uint8_t sensorPin;
	}sSensor;

	sSensor sensors[maxSensorNumber];
	int registeredSensorsNumber;
	void initSensors();
	int readSensor(sSensor& sensor);

	bool _button1Pressed;
	bool _button2Pressed;
	bool _cordIn;
	bool _redLEDOn;
	bool _greenLEDOn;
	bool _blueLEDOn;
	
	volatile bool _HMIhasChanged;

	int _rusty_duck_pos;

	TM1637Display scoreDisplay;
};

const uint8_t SEG_ENAC[] = {
		SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,  // E
		SEG_E | SEG_G | SEG_C,  // n
		SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // A
		SEG_A | SEG_F | SEG_E | SEG_D
};

const uint8_t SEG_FAT[] = {
		SEG_A | SEG_F | SEG_E | SEG_G,  // F
		SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // A
		SEG_A | SEG_B | SEG_C,  // Half T
		SEG_A | SEG_F | SEG_E   // Half T
};

const uint8_t SEG_RUSTY_DUCKS[] = {
		SEG_E | SEG_G,   // r
		SEG_C | SEG_D | SEG_E,    //u
		SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,    //S
		SEG_D | SEG_E | SEG_F | SEG_G,     //t
		SEG_B | SEG_E | SEG_F | SEG_G,  //y
		0,  //space
		SEG_B | SEG_C |SEG_D | SEG_E | SEG_G,  //d
		SEG_C | SEG_D | SEG_E,  //u
		SEG_D | SEG_E | SEG_G,  //c
		SEG_A | SEG_C | SEG_E | SEG_F | SEG_G,  // k
		SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,    //S
		0,
		0,
		0,
		0
};

extern InputOutputs inputOutputs;

#endif /* INPUTOUTPUTS_H_ */
