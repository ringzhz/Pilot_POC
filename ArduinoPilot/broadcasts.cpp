//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include<Arduino.h>
#include "broadcasts.h"

extern char gpsBuf[];


void PublishGps()
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["Topic"] = "robot1";
	root["T"] = "GPS";
	root["S"] = gpsBuf;
	root.printTo(Serial); Serial.print('\n');
}

