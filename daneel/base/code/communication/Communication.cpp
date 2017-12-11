/*
 * Communication.cpp
 *
 *  Created on: 4 déc. 2017
 *      Author: guilhem
 */

#include "Communication.h"

namespace std {

Communication::Communication(HardwareSerial serial, uint32_t baudrate): odomReportIndex(0), lastOdomReportIndexAcknowledged(0), cumulateddx(0),
		cumulateddy(0), cumulateddtheta(0), serial(serial){
	this->serial.begin(baudrate);

}

Communication::~Communication() {
	// TODO Auto-generated destructor stub
}

int Communication::sendIHMState(const bool cordState, const bool button1State, const bool button2State,
		const bool redLedState, const bool greenLedState, const bool blueLedState){
	sMessageUp msg;
	uint8_t hmiState = cordState << 8 | button1State << 7 | button2State << 6 | redLedState << 5 | greenLedState << 4 |
			blueLedState << 3;
	msg.upMsgType = Communication::HMI_STATE;
	msg.upData.hmiStateMsg.HMIState = hmiState;

	return sendUpMessage(msg);
}

int Communication::sendActuatorState(const int actuatorId, const int actuatorState){
	sMessageUp msg;
	msg.upMsgType = Communication::ACTUATOR_STATE;

	if(actuatorId < 0 || actuatorState > 255){
		return -10;
	}
	if (actuatorState < 0 || actuatorState > 65535){
		return -11;
	}
	msg.upData.actuatorStateMsg.actuatorId = actuatorId;
	msg.upData.actuatorStateMsg.actuatorValue = actuatorState;

	return sendUpMessage(msg);
}

int Communication::sendOdometryReport(const int dx, const int dy, const double dtheta){
	sMessageUp msg;
	cumulateddx += dx;
	cumulateddy += dy;
	cumulateddtheta += dtheta;
	odomReportIndex = (odomReportIndex + 1) % 256;
	int msgdx = cumulateddx + linearOdomToMsgAdder, msgdy = cumulateddy + linearOdomToMsgAdder;
	int msgdtheta = (cumulateddtheta + radianToMsgAdder) * radianToMsgFactor;

	if (cumulateddx < 0 || cumulateddx > 65535){
		return -10;
	}
	if (cumulateddy < 0 || cumulateddy > 65535){
		return -11;
	}
	if (cumulateddtheta < 0 || cumulateddtheta > 65535){
		return -12;
	}
	msg.upMsgType = Communication::ODOM_REPORT;
	msg.upData.odomReportMsg.previousReportId = lastOdomReportIndexAcknowledged;
	msg.upData.odomReportMsg.newReportId = odomReportIndex;
	msg.upData.odomReportMsg.dx = msgdx;
	msg.upData.odomReportMsg.dy = msgdy;
	msg.upData.odomReportMsg.dtheta = msgdtheta;

	return sendUpMessage(msg);
}

int Communication::sendUpMessage(const sMessageUp& msg){
	uRawMessageUp rawMessage;
	for (int i = 0; i < upMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageUp = msg;
	rawMessage.messageUp.checksum = computeUpChecksum(msg);
	rawMessage.messageUp.upMsgId = upMessageIndex;
	upMessageIndex++;
	serial.write(rawMessage.bytes, upMsgMaxSize);
	return 0;
}

uint8_t Communication::computeUpChecksum(const sMessageUp& msg){
	uRawMessageUp rawMessage;
	unsigned char checksum = 0;
	for (int i = 0; i < upMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageUp = msg;
	for (int i = upMsgHeaderSize; i < upMsgMaxSize; i++){
		checksum = checksum ^ rawMessage.bytes[i];
	}
	return checksum;
}

} /* namespace std */
