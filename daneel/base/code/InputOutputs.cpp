/*
 * InputOuputs.cpp
 *
 *  Created on: 22 déc. 2017
 *      Author: fabien
 */

#include <InputOutputs.h>
#include "DynamixelSerial5.h"
#include "params.h"
#include "communication/Communication.h"

using namespace fat;

InputOutputs inputOutputs = InputOutputs();

void ioHMIhasChanged();

InputOutputs::InputOutputs(): registeredSensorsNumber(0),_button1Pressed(false),
		_button2Pressed(false), _cordIn(false), _redLEDOn(false), _greenLEDOn(false),
		_blueLEDOn(false), _HMIhasChanged(false),
		scoreDisplay(SCORE_DISPLAY_CLK, SCORE_DISPLAY_DIO), ballDetectorGreen(BALL_DETECTOR_GREEN),
		ballDetectorOrange(BALL_DETECTOR_ORANGE){

}

InputOutputs::~InputOutputs() {

}

void InputOutputs::init() {
	Dynamixel.begin(1000000, DYNAMIXEL_CONTROL);
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);
	pinMode(CORD, INPUT_PULLUP);
	pinMode(BUTTON1, INPUT_PULLUP);
	pinMode(BUTTON2, INPUT_PULLUP);
	pinMode(WATER_CANNON_GREEN, OUTPUT);
	pinMode(WATER_CANNON_ORANGE, OUTPUT);
	analogWrite(WATER_CANNON_GREEN, 0);
	analogWrite(WATER_CANNON_ORANGE, 0);
	pinMode(BEE_ARM_GREEN, OUTPUT);
	pinMode(BEE_ARM_ORANGE, OUTPUT);
	beeArmGreen.attach(BEE_ARM_GREEN);
	beeArmOrange.attach(BEE_ARM_ORANGE);
	beeArmGreen.write(90);
	beeArmOrange.write(90);
	attachInterrupt(digitalPinToInterrupt(CORD), ioHMIhasChanged, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON1), ioHMIhasChanged, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON2), ioHMIhasChanged, CHANGE);

	initSensors();

	HMISetLedColor(0, 0, 0);
}

void InputOutputs::initSensors(){
	sSensor battery_sig, battery_pwr, ball_detector_green, ball_detector_orange;
	pinMode(BAT_SIG, INPUT);
	pinMode(BAT_POW, INPUT);
	battery_sig.sensorType = sSensor::ANALOG;
	battery_pwr.sensorType = sSensor::ANALOG;
	battery_sig.sensorId = sSensor::BATTERY_SIG;
	battery_pwr.sensorId = sSensor::BATTERY_POW;
	battery_sig.sensorPin = BAT_SIG;
	battery_pwr.sensorPin = BAT_POW;
	battery_sig.sensorReadState = sSensor::STOPPED;
	battery_pwr.sensorReadState = sSensor::STOPPED;
	battery_sig.lastReadTime = 0;
	battery_pwr.lastReadTime = 0;
	battery_sig.lastReadValue = 0;
	battery_pwr.lastReadValue = 0;
	sensors[registeredSensorsNumber++] = battery_sig;
	sensors[registeredSensorsNumber++] = battery_pwr;

	ball_detector_green = {sSensor::BALL_DETECTOR, sSensor::BALL_DETECTOR_GREEN, sSensor::STOPPED,
			0, 0, BALL_DETECTOR_GREEN};
	sensors[registeredSensorsNumber++] = ball_detector_green;

	ball_detector_orange = {sSensor::BALL_DETECTOR, sSensor::BALL_DETECTOR_ORANGE, sSensor::STOPPED,
		0, 0, BALL_DETECTOR_ORANGE};
	sensors[registeredSensorsNumber++] = ball_detector_orange;

	// { sensorType, sensorId, sensorReadState, lastReadTime, lastReadValue, sensorPin}
//	irCubeLeft.sensorType = sSensor::ANALOG;
//	irCubeLeft.sensorId = 0;
//	irCubeLeft.sensorPin = IR_CUBES_LEFT;
//	irCubeLeft.sensorReadState = STOPPED;
//	irCubeLeft.lastReadTime = 0;
//	irCubeLeft.lastReadValue = 0;
//	sensors[registeredSensorsNumber] = irCubeLeft;
//	registeredSensorsNumber++;
}

void InputOutputs::reset(){
	for (int i = 0; i < registeredSensorsNumber; i++){
		sSensor s = sensors[i];
		s.lastReadTime = 0;
		s.lastReadValue = 0;
		s.sensorReadState = sSensor::STOPPED;
	}
	ballDetectorGreen.reset();
	ballDetectorOrange.reset();
}

void InputOutputs::run(){
	if(isHmIhasChanged()) {
		HMISendState();
	}
	for (int i = 0; i < registeredSensorsNumber; i++){
		sSensor* s = &sensors[i];
		int value;
		switch (s->sensorReadState){
		case sSensor::STOPPED:
			continue;
			break;
		case sSensor::ON_CHANGE:
			value = readSensor(*s);
			if (value != s->lastReadValue){
				s->lastReadValue = value;
				s->lastReadTime = millis();
				communication.sendSensorValue(s->sensorId, s->lastReadValue);
			}
			continue;
			break;
		case sSensor::PERIODIC:
			long now = millis();
			Serial.println(now);
			if (now - s->lastReadTime >= sensorPeriodicTime){
				s->lastReadValue = readSensor(*s);
				s->lastReadTime = now;
				communication.sendSensorValue(s->sensorId, s->lastReadValue);
			}
			break;
		}
	}
}

int InputOutputs::readSensor(sSensor& sensor){
	switch (sensor.sensorType){
	case sSensor::ANALOG:
		return analogRead(sensor.sensorPin);
		break;
	case sSensor::DIGITAL:
		return digitalRead(sensor.sensorPin);
		break;
	case sSensor::DYNAMIXEL_POSITION:
		return Dynamixel.readPosition(sensor.sensorPin);
		break;
	case sSensor::BALL_DETECTOR:
		if (sensor.sensorId == sSensor::BALL_DETECTOR_GREEN){
			return ballDetectorGreen.getNumberOfBalls();
		}else{
			return ballDetectorOrange.getNumberOfBalls();
		}
		break;
	}
	return 0;
}

void InputOutputs::updateBallDetector(){
	ballDetectorGreen.detect();
	ballDetectorOrange.detect();
}

bool InputOutputs::HMIGetButton1State() {
	_button1Pressed = digitalRead(BUTTON1);
	return _button1Pressed;
}

bool InputOutputs::HMIGetButton2State() {
	_button2Pressed = digitalRead(BUTTON2);
	return _button2Pressed;
}

bool InputOutputs::HMIGetCordState() {
	_cordIn = digitalRead(CORD);
	return _cordIn;
}

void InputOutputs::HMISetLedColor(int red, int green, int blue){
	_redLEDOn = red;
	_greenLEDOn = green;
	_blueLEDOn = blue;
	digitalWrite(LED_RED, _redLEDOn ? HIGH : LOW);
	digitalWrite(LED_GREEN, _greenLEDOn ? HIGH : LOW);
	digitalWrite(LED_BLUE, _blueLEDOn ? HIGH : LOW);
	HMISendState();
}

void InputOutputs::HMISendState(){
	_cordIn = digitalRead(CORD);
	_button1Pressed = digitalRead(BUTTON1);
	_button2Pressed = digitalRead(BUTTON2);
	_HMIhasChanged = false;
	communication.sendIHMState(_cordIn, _button1Pressed, _button2Pressed, _redLEDOn, _greenLEDOn, _blueLEDOn);
}

void ioHMIhasChanged(){
	inputOutputs.setHmIhasChanged(true);
}

void InputOutputs::deliverWater(bool enable, int dynamixelId, bool direction) {
	if(enable) {
		Dynamixel.setEndless(2, true);
		Dynamixel.turn(dynamixelId, direction, 1023);
	}
	else {
		Dynamixel.turn(dynamixelId, direction, 0);
	}
}

void InputOutputs::handleActuatorMessage(int actuatorId, int actuatorCommand){
	switch(actuatorId){
	case eMsgActuatorId::WATER_DELIVERING_DYNAMIXEL_GREEN:
		deliverWater(actuatorCommand, WATER_DELIVERER_GREEN, DYNA_TURN_CCW);
		break;
	case eMsgActuatorId::WATER_DELIVERING_DYNAMIXEL_ORANGE:
		deliverWater(actuatorCommand, WATER_DELIVERER_ORANGE, DYNA_TURN_CW);
		break;
	case eMsgActuatorId::WATER_CANNON_DC_MOTOR_GREEN:
		analogWrite(WATER_CANNON_GREEN, actuatorCommand);
		break;
	case eMsgActuatorId::WATER_CANNON_DC_MOTOR_ORANGE:
		analogWrite(WATER_CANNON_ORANGE, actuatorCommand);
		break;
	case eMsgActuatorId::SCORE_COUNTER:
		scoreDisplay.setBrightness(7, true);
		if (actuatorCommand <= 9999){
			scoreDisplay.showNumberDecEx(actuatorCommand, 0);
		}
		else if(actuatorCommand <= 19999){
			scoreDisplay.showNumberDecEx(actuatorCommand - 10000, 255, true);
		}
		else if (actuatorCommand == 20001){
			scoreDisplay.setSegments(SEG_ENAC);
		}else if (actuatorCommand == 20002){
			scoreDisplay.setSegments(SEG_FAT);
		}else{
			scoreDisplay.setBrightness(0, false);
		}
		break;
	case eMsgActuatorId::BEE_ARM_SERVO_GREEN:
		beeArmGreen.write(actuatorCommand);
		break;
	case eMsgActuatorId::BEE_ARM_SERVO_ORANGE:
		beeArmOrange.write(actuatorCommand);
		break;
	default:
		break;
	}
}

void InputOutputs::handleSensorCommand(int sensorId, int sensorCommand){
	for (int i = 0; i < registeredSensorsNumber; i++){
		if (sensors[i].sensorId == sensorId){
			sensors[i].sensorReadState = static_cast<sSensor::eSensorReadState>(sensorCommand);
			break;
		}
	}
}

