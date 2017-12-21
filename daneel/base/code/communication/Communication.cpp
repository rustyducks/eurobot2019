/*
 * Communication.cpp
 *
 *  Created on: 4 déc. 2017
 *      Author: guilhem
 */

#include "Communication.h"

using namespace std;
namespace fat {

Communication::Communication(HardwareSerial serial, uint32_t baudrate):serial(serial), odomReportIndex(0),
		lastOdomReportIndexAcknowledged(0),upMessageIndex(0), lastIdDownMessageRecieved(0),
		isFirstMessage(true){
	this->serial.begin(baudrate);
	for (unsigned int i = 0; i < maxNonAckMessageStored; i++){
		toBeAcknowledged[i].sendTime = 0;
	}

}

Communication::~Communication() {
	// TODO Auto-generated destructor stub
}

int Communication::sendIHMState(const bool cordState, const bool button1State, const bool button2State,
		const bool redLedState, const bool greenLedState, const bool blueLedState){
	sMessageUp msg;
	uint8_t hmiState = cordState << 7 | button1State << 6 | button2State << 5 | redLedState << 4 | greenLedState << 3 |
			blueLedState << 2;
	msg.upMsgType = Communication::HMI_STATE;
	msg.upData.hmiStateMsg.HMIState = hmiState;

	return sendUpMessage(msg);
}

int Communication::sendActuatorState(const int actuatorId, const int actuatorState){
	sMessageUp msg;
	msg.upMsgType = Communication::ACTUATOR_STATE;

	if(actuatorId < 0 || actuatorId > 255){
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
	sOdomReportStorage odomReport;
	int cumuleddx = dx, cumuleddy = dy;
	double cumuleddtheta = dtheta;

	odomReportIndex = (odomReportIndex + 1) % 256;
	odomReport = {(uint8_t)odomReportIndex, (double)dx, (double)dy, (double)dtheta};

	for (sOdomReportStorage odom: nonAcknowledgedOdomReport){
		if ((odom.odomId - lastOdomReportIndexAcknowledged) % 256 > 0 &&
				(odomReportIndex - odom.odomId) % 256 > 0){
			// we must have lastOdomAck < odomId < odomReportIndex to add it in the report (but maybe check if the mod. is correct)
			cumuleddx += odom.dx;
			cumuleddy += odom.dy;
			cumuleddtheta += odom.dtheta;
		}
	}

	nonAcknowledgedOdomReport.push_back(odomReport);

	int msgdx = cumuleddx + linearOdomToMsgAdder, msgdy = cumuleddy + linearOdomToMsgAdder;
	int msgdtheta = (cumuleddtheta + radianToMsgAdder) * radianToMsgFactor;

	if (cumuleddx < 0 || cumuleddx > 65535){
		return -10;
	}
	if (cumuleddy < 0 || cumuleddy > 65535){
		return -11;
	}
	if (cumuleddtheta < 0 || cumuleddtheta > 65535){
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

int Communication::storeNewSentMessage(unsigned long time, const uRawMessageUp msg){
	bool stored = false;
	sUpMessageStorage toStore = {time, msg};
	for (unsigned int i = 0; i < maxNonAckMessageStored; i++){
		if (toBeAcknowledged[i].sendTime == 0){
			toBeAcknowledged[i] = toStore;
			stored = true;
			break;
		}
	}
	if (stored){
		return 0;
	}
	return 1;
}

int Communication::removeAcknowledgedMessage(uint8_t acknowledgedId){
	bool isRemoved = false;
	for (unsigned int i=0; i < maxNonAckMessageStored; i++){
		if (toBeAcknowledged[i].message.messageUp.upMsgId == acknowledgedId){
			toBeAcknowledged[i].sendTime = 0;
			isRemoved = true;
		}
	}
	return isRemoved;
}

int Communication::sendUpMessage(const sMessageUp& msg){
	uRawMessageUp rawMessage;
	int storageSuccess;
	for (int i = 0; i < upMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageUp = msg;
	rawMessage.messageUp.checksum = computeUpChecksum(msg);
	rawMessage.messageUp.upMsgId = upMessageIndex;
	upMessageIndex++;
	serial.write(rawMessage.bytes, upMsgMaxSize);
	storageSuccess = storeNewSentMessage(millis(), rawMessage);
	// Todo : Handle resend on timeout if ack is not recieved -> Delete stored message on ack recieve and update send time on resend.
	return storageSuccess;
}

uint8_t Communication::computeUpChecksum(const sMessageUp& msg){
	uRawMessageUp rawMessage;
	unsigned char checksum = 0;
	for (int i = 0; i < upMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageUp = msg;
	for (int i = upMsgHeaderSize; i < upMsgMaxSize; i++){
		checksum = (checksum ^ rawMessage.bytes[i]) % 256;
	}
	return checksum;
}

uint8_t Communication::computeDownChecksum(const sMessageDown& msg){
	uRawMessageDown rawMessage;
	unsigned char checksum = 0;
	for (int i = 0; i < downMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageDown = msg;
	for (int i = downMsgHeaderSize; i < downMsgMaxSize; i++){
		checksum = (checksum ^ rawMessage.bytes[i]) % 256;
	}
	return checksum;
}

int Communication::registerActuatorCommandCallback(ActuatorCommandCallback callback){
	if (actuatorMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	actuatorMsgCallbacks.cb[actuatorMsgCallbacks.index] = callback;
	actuatorMsgCallbacks.index++;
	return 0;
}

int Communication::registerHMICommandCallback(HMICommandCallback callback){
	if (HMIMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	HMIMsgCallbacks.cb[HMIMsgCallbacks.index] = callback;
	HMIMsgCallbacks.index++;
	return 0;
}

int Communication::registerSpeedCommandCallback(SpeedCommandCallback callback){
	if (speedMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	speedMsgCallbacks.cb[speedMsgCallbacks.index] = callback;
	speedMsgCallbacks.index++;
	return 0;
}

int Communication::registerRepositionningCallback(RepositionningCallback callback){
	if (repositioningCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	repositioningCallbacks.cb[repositioningCallbacks.index] = callback;
	repositioningCallbacks.index++;
	return 0;
}

void Communication::recieveMessage(const sMessageDown& msg){
	vector<sOdomReportStorage>::iterator nonAckOdomReportItr;
	SpeedCommand speedCommand;
	ActuatorCommand actuatorCommand;
	HMICommand hmiCommand;

	switch(msg.downMsgType){
	case ACK_DOWN:
		removeAcknowledgedMessage(msg.downData.ackMsg.ackUpMsgId);
		break;
	case ACK_ODOM_REPORT:
		removeAcknowledgedMessage(msg.downData.ackOdomReportMsg.ackUpMsgId);

		nonAckOdomReportItr = nonAcknowledgedOdomReport.begin();
		while(nonAckOdomReportItr != nonAcknowledgedOdomReport.end()){
			if ((nonAckOdomReportItr->odomId -
					msg.downData.ackOdomReportMsg.ackOdomReportId) % 256 < 0){
				nonAckOdomReportItr = nonAcknowledgedOdomReport.erase(nonAckOdomReportItr);
			}else{
				nonAckOdomReportItr++;
			}
		}

		break;
	case SPEED_CMD:
		speedCommand.vx = msg.downData.speedCmdMsg.vx - linearSpeedToMsgAdder;  // Todo : Maybe scale linear speed
 		speedCommand.vy = msg.downData.speedCmdMsg.vy - linearSpeedToMsgAdder;
 		speedCommand.vtheta = msg.downData.speedCmdMsg.vtheta / radianToMsgFactor - radianToMsgAdder;
 		for (unsigned int i=0; i < speedMsgCallbacks.index; i++){
			speedMsgCallbacks.cb[i](speedCommand);
		}
		break;
	case ACTUATOR_CMD:
		actuatorCommand.actuatorId = msg.downData.actuatorCmdMsg.actuatorId;
		actuatorCommand.actuatorCommand = msg.downData.actuatorCmdMsg.actuatorCmd;
		for (unsigned int i=0; i < actuatorMsgCallbacks.index; i++){
			actuatorMsgCallbacks.cb[i](actuatorCommand);
		}
		break;
	case HMI_CMD:
		hmiCommand.redLedCommand = msg.downData.hmiCmdMsg.hmiCmd & hmiCommandRedMask;
		hmiCommand.greenLedCommand = msg.downData.hmiCmdMsg.hmiCmd & hmiCommandGreenMask;
		hmiCommand.blueLedCommand = msg.downData.hmiCmdMsg.hmiCmd & hmiCommandBlueMask;
		for (unsigned int i=0; i < HMIMsgCallbacks.index; i++){
			HMIMsgCallbacks.cb[i](hmiCommand);
		}
		break;
	}
}

void Communication::checkMessages(){
	uRawMessageDown rawDataDown;
	uRawMessageUp rawUpAckMessage;
	if (serial.available()) { //If there is some data waiting in the buffer
		if (serial.available() >= downMsgMaxSize) { //Read all the data in the buffer (asserting raspi is sending at max one message per teensy loop)
			for (int i = 0; i < downMsgMaxSize; i++){
				rawDataDown.bytes[i] = serial.read();
			}
#if DEBUG_COMM
			char tmp[16];
			for (int debug = 0; debug < downMsgMaxSize; debug++){
				sprintf(tmp, "%.2X",rawDataDown.bytes[debug]);
				Serial.print(tmp);
			}
			Serial.println();
#endif
			if (computeDownChecksum(rawDataDown.messageDown) == rawDataDown.messageDown.checksum){
				rawUpAckMessage.messageUp.upMsgType = ACK_UP;
				rawUpAckMessage.messageUp.upData.ackMsg.ackDownMsgId = rawDataDown.messageDown.downMsgId;
				serial.write(rawUpAckMessage.bytes, upMsgMaxSize);


				if (isFirstMessage || //If it is the first message, accept it
						/*raw_data_down.msg.type == RESET ||*/
						((rawDataDown.messageDown.downMsgId - lastIdDownMessageRecieved)%256>0
								&& (rawDataDown.messageDown.downMsgId - lastIdDownMessageRecieved)%256<128)) { //Check if the message has a id bigger than the last recevied
					isFirstMessage = false;
					lastIdDownMessageRecieved = rawDataDown.messageDown.downMsgId;

					recieveMessage(rawDataDown.messageDown);

				}
			}/* else {
				raw_ack_message.msg.type = NON_ACK;
				raw_ack_message.msg.down_id = raw_data_down.msg.id;
				HWSERIAL.write(raw_ack_message.data, MSG_UP_MAX_SIZE);
				return 0;*/

		}
	} /*else {
		return 0; //Serial is empty :'(
	}*/
}

} /* namespace std */
