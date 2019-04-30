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
		scoreDisplay(SCORE_DISPLAY_CLK, SCORE_DISPLAY_DIO){

}

InputOutputs::~InputOutputs() {

}

void InputOutputs::init() {
	//Dynamixel.begin(1000000, DYNAMIXEL_CONTROL);
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);
	pinMode(CORD, INPUT_PULLUP);
	pinMode(BUTTON1, INPUT_PULLUP);
	pinMode(BUTTON2, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CORD), ioHMIhasChanged, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON1), ioHMIhasChanged, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON2), ioHMIhasChanged, CHANGE);

	initSensors();

	HMISetLedColor(0, 0, 0);

	pinMode(LIDAR_SPEED, OUTPUT);
	analogWrite(LIDAR_SPEED, LIDAR_BASE_PWM);
}

void InputOutputs::initSensors(){
	sSensor battery_sig, battery_pwr;
	pinMode(BAT_SIG, INPUT);
	battery_sig.sensorType = sSensor::ANALOG;
	battery_sig.sensorId = sSensor::BATTERY_SIG;
	battery_sig.sensorPin = BAT_SIG;
	battery_sig.sensorReadState = sSensor::STOPPED;
	battery_sig.lastReadTime = 0;
	battery_sig.lastReadValue = 0;

	pinMode(BAT_POW, INPUT);
	battery_pwr.sensorType = sSensor::ANALOG;
	battery_pwr.sensorId = sSensor::BATTERY_POW;
	battery_pwr.sensorPin = BAT_POW;
	battery_pwr.sensorReadState = sSensor::STOPPED;
	battery_pwr.lastReadTime = 0;
	battery_pwr.lastReadValue = 0;

	sensors[registeredSensorsNumber++] = battery_sig;
	sensors[registeredSensorsNumber++] = battery_pwr;

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
		sensors[i].lastReadTime = 0;
		sensors[i].lastReadValue = 0;
		sensors[i].sensorReadState = sSensor::STOPPED;
	}
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
		//return Dynamixel.readPosition(sensor.sensorPin);
		return 0;
		break;
	}
	return 0;
}

bool InputOutputs::HMIGetButton1State() {
	_button1Pressed = digitalRead(BUTTON1);
	return _button1Pressed;
}

bool InputOutputs::HMIGetButton2State() {
	//_button2Pressed = digitalRead(BUTTON2);
	return _button2Pressed;
}

bool InputOutputs::HMIGetCordState() {
	//_cordIn = digitalRead(CORD);
	return _cordIn;
}

void InputOutputs::HMISetLedColor(int red, int green, int blue){
	_redLEDOn = red;
	_greenLEDOn = green;
	_blueLEDOn = blue;
	analogWrite(LED_RED, red);
	analogWrite(LED_GREEN, green);
	analogWrite(LED_BLUE, blue);
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

void InputOutputs::handleActuatorMessage(int actuatorId, int actuatorCommand){
	switch(actuatorId){
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

