//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "MPU6050.h"
#include "helper_3dmath.h"
#include "PilotMotor.h"
#include "ArduinoPilot.h"
#include "pose.h"

void PublishPose()
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Pose";
	root["X"].set(X / 1000, 4);		// mm to meter
	root["Y"].set(Y / 1000, 4);
	root["H"].set(RAD_TO_DEG * H, 2);
	root.printTo(Serial); Serial.print('\n');
}

void PublishHeartbeat()
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Heartbeat";

#if 1
	root["M1Tach"].set(M1.GetTacho(), 0);  // 0 is the number of decimals to print
	root["M2Tach"].set(M2.GetTacho(), 0);
	if (AhrsEnabled)
	{
		root["Yaw"].set(ypr[0], 4);
		root["Pit"].set(ypr[1], 4);
		root["Rol"].set(ypr[2], 4);
	}
#endif

	root.printTo(Serial); Serial.print('\n');
}