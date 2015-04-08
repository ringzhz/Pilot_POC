//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "MPU6050_6Axis_MotionApps20.h"
#include "helper_3dmath.h"
#include "PilotMotor.h"
#include "ArduinoPilot.h"
#include "pose.h"

void PublishPose()
{
	StaticJsonBuffer<128> jsonBuffer;
#if 1
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Tach";
	root["M1"].set(M1.GetTacho(), 0);  // 0 is the number of decimals to print
	root["M2"].set(M2.GetTacho(), 0);
	root.printTo(Serial); Serial.print('\n');

#endif
	JsonObject& root2 = jsonBuffer.createObject();
	root2[Topic] = "robot1";
	root2["T"] = "Pose";
	root2["X"].set(X/1000, 6);		// mm to meter
	root2["Y"].set(Y/1000, 6);
	root2["H"].set(RAD_TO_DEG * H, 4);
	root2.printTo(Serial); Serial.print('\n');
}

void PublishHeartbeat()
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Heartbeat";
	//if (mpuEnabled)
	//	root["MeanGyro"].set((MeanGyroValue() + GyroOffset) / 100, 3);
	root.printTo(Serial); Serial.print('\n');
}