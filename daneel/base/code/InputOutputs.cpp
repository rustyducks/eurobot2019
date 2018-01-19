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

InputOutputs::InputOutputs(): _button1Pressed(false), _button2Pressed(false), _cordIn(false),
		_redLEDOn(false), _greenLEDOn(false), _blueLEDOn(false), _HMIhasChanged(false) {

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
	attachInterrupt(digitalPinToInterrupt(CORD), ioHMIhasChanged, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON1), ioHMIhasChanged, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON2), ioHMIhasChanged, CHANGE);

	HMISetLedColor(0, 0, 0);
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
	communication.sendIHMState(_cordIn, _button1Pressed, _button2Pressed, _redLEDOn, _greenLEDOn, _blueLEDOn);
}



void ioHMIhasChanged(){
	inputOutputs.setHmIhasChanged(true);
}

void InputOutputs::deliverWater(bool enable) {
	if(enable) {
		Dynamixel.setEndless(2, true);
		Dynamixel.turn(WATER_DELIVERER, DYNA_TURN_CCW, 1023);
	}
	else {
		Dynamixel.turn(WATER_DELIVERER, DYNA_TURN_CCW, 0);
	}
}
