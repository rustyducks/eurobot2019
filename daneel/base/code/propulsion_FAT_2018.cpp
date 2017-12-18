// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include <Arduino.h>
#include "communication/Communication.h"

using namespace fat;
Communication comm(Serial1, 115200);  // Initialisation, nanani global nanana, mais c'est juste pour la démo

void testCallback(const Communication::HMICommand msg);  // Forward declaration

void setup()
{
	Serial.begin(115200);
	//Serial1.begin(115200);
}


void loop()
{

	comm.registerHMICommandCallback(testCallback);

	while (true){
		comm.checkMessages();
	}
//Add your repeated code here
}

void testCallback(const Communication::HMICommand msg){
	Serial.print("Force rouge : ");
	Serial.print(msg.redLedCommand);
	Serial.print("\tForce verte : ");
	Serial.print(msg.greenLedCommand);
	Serial.print("\tForce bleue : ");
	Serial.println(msg.blueLedCommand);
}
