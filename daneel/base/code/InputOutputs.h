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

	void HMISetLedColor(int red, int green, int blue);

	bool HMIGetButton1State();
	bool HMIGetButton2State();

	bool HMIGetCordState();
	void HMISendState();

	void moveArmBase(int degree);
	void moveArmGripper(int degree);

	void handleActuatorMessage(int actuatorId, int actuatorCommand);
	void deliverWater(bool enable, int dynamixelId, bool direction);

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
		WATER_DELIVERING_DYNAMIXEL_GREEN = 0,
		WATER_DELIVERING_DYNAMIXEL_ORANGE = 1,
		WATER_CANNON_DC_MOTOR_GREEN = 2,
		WATER_CANNON_DC_MOTOR_ORANGE = 3,
		ARM_BASE_DYNAMIXEL = 4,
		ARM_GRIPPER_DYNAMIXEL = 5,
		SCORE_COUNTER = 6,
		BEE_ARM_SERVO_GREEN = 7,
		BEE_ARM_SERVO_ORANGE = 8
	}eMsgActuatorId;

	typedef struct{
		enum eSensorType{
			DIGITAL,
			ANALOG,
			DYNAMIXEL_POSITION
		};
		enum eSensorId{
			BATTERY_SIG = 0,
			BATTERY_POW = 1,
			ARM_BASE_POSITION = 2,
			ARM_GRIP_POSITION = 3
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

	TM1637Display scoreDisplay;
	Servo beeArmGreen, beeArmOrange;
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

extern InputOutputs inputOutputs;

#endif /* INPUTOUTPUTS_H_ */
