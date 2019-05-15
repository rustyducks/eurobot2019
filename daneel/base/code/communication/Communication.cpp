/*
 * Communication.cpp
 *
 *  Created on: 4 déc. 2017
 *      Author: guilhem
 */

#include "Communication.h"
#include "MotorControl.h"

using namespace std;
namespace fat {

Communication communication = Communication(Serial1, RASPI_COMMUNICATION_BAUDRATE);


Communication::Communication(HardwareSerial& serial, uint32_t baudrate):serial(serial), baudrate(baudrate),
		upMessageIndex(0), lastIdDownMessageRecieved(0), isFirstMessage(true), state(WAITING){
	for (unsigned int i = 0; i < maxNonAckMessageStored; i++){
		toBeAcknowledged[i].sendTime = 0;
	}
	timeLastSpeedMessage = 0;
}

Communication::~Communication() {
	// TODO Auto-generated destructor stub
}

void Communication::init(){
	this->serial.begin(baudrate);
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

int Communication::sendSensorValue(const int sensorId, const int sensorValue){
	sMessageUp msg;
	msg.upMsgType = Communication::SENSOR_VALUE;
	if (sensorId < 0 || sensorId > 255){
		return -10;
	}
	if (sensorValue < 0 || sensorValue > 65535){
		return -11;
	}
	msg.upData.sensorValueMsg.sensorId = sensorId;
	msg.upData.sensorValueMsg.sensorValue = sensorValue;

	return sendUpMessage(msg);
}

int Communication::sendOdometryPosition(const double x, const double y, const double theta){
	sMessageUp msg;

	int msgx = round((x + linearPositionToMsgAdder) * linearPositionToMsgFactor);
	int msgy = round((y + linearPositionToMsgAdder) * linearPositionToMsgFactor);
	int msgtheta = round((theta + radianToMsgAdder) * radianToMsgFactor);

	if (msgx < 0 || msgx > 65535){
		return -10;
	}
	if (msgy < 0 || msgy > 65535){
		return -11;
	}
	if (msgtheta < 0 || msgtheta > 65535){
		return -12;
	}
	msg.upMsgType = Communication::ODOM_REPORT;
	msg.upData.odomReportMsg.x = msgx;
	msg.upData.odomReportMsg.y = msgy;
	msg.upData.odomReportMsg.theta = msgtheta;

#if DEBUG_COMM
	Serial.print("Sending Odom Report: ");
	Serial.print(x);
	Serial.print("; ");
	Serial.print(y);
	Serial.print("; ");
	Serial.print(theta);
#endif

	return sendUpMessage(msg);
}

int Communication::sendSpeedReport(const double vx, const double vy, const double vtheta, const bool drifting_left, const bool drifting_right){
	sMessageUp msg;
	int msgvx = round((vx + linearSpeedToMsgAdder) * linearSpeedToMsgFactor);
	int msgvy = round((vy + linearSpeedToMsgAdder) * linearSpeedToMsgFactor);
	int msgvtheta = round((vtheta + angularSpeedToMsgAdder) * angularSpeedToMsgFactor);
	int msgdrifting = int(drifting_right) << 1 + int(drifting_left);

	if (msgvx < 0 || msgvx > 65535){
		return -10;
	}
	if (msgvy < 0 || msgvy > 65535){
		return -11;
	}
	if (msgvtheta < 0 || msgvtheta > 65535){
		return -12;
	}
	msg.upMsgType = Communication::SPEED_REPORT;
	msg.upData.speedReportMsg.vx = msgvx;
	msg.upData.speedReportMsg.vy = msgvy;
	msg.upData.speedReportMsg.vtheta = msgvtheta;
	msg.upData.speedReportMsg.drifting = msgdrifting;

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
	byte txPacket[100];
	int s = 0;
	int storageSuccess;

	rawMessage.messageUp = msg;
	rawMessage.messageUp.dataSize = getMessageSize(msg);
	rawMessage.messageUp.checksum = computeUpChecksum(msg);
	rawMessage.messageUp.upMsgId = upMessageIndex;

	txPacket[s++] = 0xFF;
	txPacket[s++] = 0xFF;
	for (int i = 0; i < rawMessage.messageUp.dataSize + upMsgHeaderSize; i++){
		txPacket[s++] = rawMessage.bytes[i];
	}


#if DEBUG_COMM
	Serial.println("Sending packet:");
	for (int i = 0; i < s; i++){
		Serial.print(txPacket[i]);
	}
	Serial.println();
#endif

	serial.write(txPacket, s);

	if (msg.upMsgType == ACK_DOWN){
		return 0;
	}
	upMessageIndex++;
	storageSuccess = storeNewSentMessage(millis(), rawMessage);
	// Todo : Handle resend on timeout if ack is not recieved -> Delete stored message on ack recieve and update send time on resend.
	return storageSuccess;
}

uint8_t Communication::getMessageSize(const sMessageUp& msg){
	switch(msg.upMsgType){
	case ACK_DOWN:
		return sizeof(sAckDown);
		break;
	case ODOM_REPORT:
		return sizeof(sOdomReportMsg);
		break;
	case HMI_STATE:
		return sizeof(sHMIStateMsg);
		break;
	case SENSOR_VALUE:
		return sizeof(sSensorValueMsg);
		break;
	case SPEED_REPORT:
		return sizeof(sSpeedReportMsg);
		break;
	default:
		return 0;
		break;
	}
}

uint8_t Communication::computeUpChecksum(const sMessageUp& msg){
	uRawMessageUp rawMessage;
	unsigned char checksum = 0;
	rawMessage.messageUp = msg;
	for (int i = upMsgHeaderSize; i < msg.dataSize + upMsgHeaderSize; i++){
		checksum = (checksum ^ rawMessage.bytes[i]) % 256;
	}
	return checksum;
}

uint8_t Communication::computeDownChecksum(const sMessageDown& msg){
	uRawMessageDown rawMessage;
	unsigned char checksum = 0;
	rawMessage.messageDown = msg;
	for (int i = downMsgHeaderSize; i < msg.dataSize + downMsgHeaderSize; i++){
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

int Communication::registerResetCallback(ResetCallback callback){
	if (resetCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	resetCallbacks.cb[resetCallbacks.index] = callback;
	resetCallbacks.index++;
	return 0;
}

int Communication::registerSensorCommandCallback(SensorCommandCallback callback){
	if (sensorMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	sensorMsgCallbacks.cb[sensorMsgCallbacks.index] = callback;
	sensorMsgCallbacks.index++;
	return 0;
}

void Communication::recieveMessage(const sMessageDown& msg){
	SpeedCommand speedCommand;
	ActuatorCommand actuatorCommand;
	HMICommand hmiCommand;
	Repositionning repositioning;
	SensorCommand sensorCommand;

	switch(msg.downMsgType){
	case ACK_UP:
		removeAcknowledgedMessage(msg.downData.ackMsg.ackUpMsgId);
		break;

	case SPEED_CMD:
		speedCommand.vx = msg.downData.speedCmdMsg.vx / linearSpeedToMsgFactor - linearSpeedToMsgAdder;
 		speedCommand.vy = msg.downData.speedCmdMsg.vy /linearSpeedToMsgFactor - linearSpeedToMsgAdder;
 		speedCommand.vtheta = msg.downData.speedCmdMsg.vtheta / angularSpeedToMsgFactor - angularSpeedToMsgAdder;
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
		hmiCommand.redLedCommand = ((msg.downData.hmiCmdMsg.hmiCmd & hmiCommandRedMask) >> 5) * 255 / 7;
		hmiCommand.greenLedCommand = ((msg.downData.hmiCmdMsg.hmiCmd & hmiCommandGreenMask) >> 2) * 255 / 7;
		hmiCommand.blueLedCommand = (msg.downData.hmiCmdMsg.hmiCmd & hmiCommandBlueMask) * 255 / 3;
		for (unsigned int i=0; i < HMIMsgCallbacks.index; i++){
			HMIMsgCallbacks.cb[i](hmiCommand);
		}
		break;
	case RESET:
		for (unsigned int i = 0; i < resetCallbacks.index; i++){
			resetCallbacks.cb[i]();
		}
		break;
	case REPOSITIONING:
		repositioning.x = msg.downData.repositioningMsg.xRepositioning / linearPositionToMsgFactor - linearPositionToMsgAdder;
		repositioning.y = msg.downData.repositioningMsg.yRepositioning / linearPositionToMsgFactor - linearPositionToMsgAdder;
		repositioning.theta = msg.downData.repositioningMsg.thetaRepositioning / radianToMsgFactor - radianToMsgAdder;
		for (unsigned int i = 0; i < repositioningCallbacks.index; i++){
			repositioningCallbacks.cb[i](repositioning);
		}
		break;
	case PID_TUNING:
		motorControl.setKpSpeed(msg.downData.pidTuningMsg.kp_linear / 1000.0);
		motorControl.setKiSpeed(msg.downData.pidTuningMsg.ki_linear / 1000.0);
		motorControl.setKdSpeed(msg.downData.pidTuningMsg.kd_linear / 1000.0);
		motorControl.setKpOmega(msg.downData.pidTuningMsg.kp_angular / 100.0);
		motorControl.setKiOmega(msg.downData.pidTuningMsg.ki_angular / 100.0);
		motorControl.setKdOmega(msg.downData.pidTuningMsg.kd_angular / 100.0);
		break;
	case SENSOR_CMD:
		sensorCommand.sensorId = msg.downData.sensorCmdMsg.sensorId;
		sensorCommand.sensorCommand = msg.downData.sensorCmdMsg.sensorState;
		for (unsigned int i=0; i < sensorMsgCallbacks.index; i++){
			sensorMsgCallbacks.cb[i](sensorCommand);
		}
		break;
	}
}

void Communication::checkMessages(){
	sMessageUp upAckMessage;
	if (state == WAITING){
		if (serial.available() >= downMsgHeaderSize + 2) { //If there is some data waiting in the buffer
			byte incoming = serial.read();
			if (incoming == 0xFF && serial.peek() == 0xFF) {
				serial.read();  // The second 0xFF that we have peeked
				for (int i=0; i < downMsgHeaderSize; i++){
					receivingMsg.bytes[i] = serial.read();
#if DEBUG_COMM
					Serial.print((int)receivingMsg.bytes[i]);
					Serial.print(' ');
#endif
				}
				//Serial.println();
				if (receivingMsg.messageDown.downMsgType > SENSOR_CMD){
#if DEBUG_COMM
					Serial.print("Invalid down message type: ");
					Serial.println(receivingMsg.messageDown.downMsgType);
#endif
					serial.flush();
					return;
				}
				if (receivingMsg.messageDown.dataSize > downMsgMaxSize - downMsgHeaderSize){
#if DEBUG_COMM
					Serial.print("Invalid down message size: ");
					Serial.println(receivingMsg.messageDown.dataSize);
#endif
					serial.flush();
					return;
				}
				state = HEADER_RECEIVED;
			}
		}
	}

	if (state == HEADER_RECEIVED){
		if (serial.available() >= receivingMsg.messageDown.dataSize){
			for (int i=0; i < receivingMsg.messageDown.dataSize; i++){
				receivingMsg.bytes[i + downMsgHeaderSize] = serial.read();
			}
#if DEBUG_COMM
			char tmp[16];
			for (int debug = 0; debug < downMsgHeaderSize + receivingMsg.messageDown.dataSize; debug++){
				sprintf(tmp, "%.2X",receivingMsg.bytes[debug]);
				Serial.print(tmp);
				Serial.print(" ");
			}
			Serial.println();
#endif
			state = WAITING;
			if (computeDownChecksum(receivingMsg.messageDown) == receivingMsg.messageDown.checksum){
				upAckMessage.upMsgType = ACK_DOWN;
				upAckMessage.upData.ackMsg.ackDownMsgId = receivingMsg.messageDown.downMsgId;
				sendUpMessage(upAckMessage);
				if (isFirstMessage || //If it is the first message, accept it
						receivingMsg.messageDown.downMsgType == RESET ||
						((receivingMsg.messageDown.downMsgId - lastIdDownMessageRecieved + 256 )%256>0
								&& (receivingMsg.messageDown.downMsgId - lastIdDownMessageRecieved)%256<128)) { //Check if the message has a id bigger than the last recevied
					isFirstMessage = false;
					lastIdDownMessageRecieved = receivingMsg.messageDown.downMsgId;

					recieveMessage(receivingMsg.messageDown);
				}
			}
		}
	}
}

void Communication::reset(){
	for (unsigned int i = 0; i < maxNonAckMessageStored; i++){
		toBeAcknowledged[i].sendTime = 0;
	}
	state = WAITING;
	upMessageIndex = 0;
	lastIdDownMessageRecieved = 0;
	isFirstMessage = true;
	while(serial.available()){
		serial.read();
	}
}

} /* namespace fat */
