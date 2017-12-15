// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include <Arduino.h>
#include "communication/Communication.h"

using namespace fat;
Communication comm(Serial2, 115200);  // Initialisation, nanani global nanana, mais c'est juste pour la démo

void testCallback(const Communication::HMICommand& msg);  // Forward declaration

void setup()
{
	Serial1.begin(115200);
}


void loop()
{

	comm.registerRecieveHMICommandCallback(testCallback);

	while (true){
		comm.checkMessages();
	}
//Add your repeated code here
}

void testCallback(const Communication::HMICommand& msg){
	Serial1.print("Force rouge : ");
	Serial1.print(msg.redLedCommand);
	Serial1.print("\tForce verte : ");
	Serial1.print(msg.greenLedCommand);
	Serial1.print("\tForce bleue : ");
	Serial1.println(msg.blueLedCommand);
}
