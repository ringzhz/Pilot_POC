//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <ArduinoJson.h>

#include "MPU6050.h"
#include "helper_3dmath.h"
#include "PilotMotor.h"
#include "ArduinoPilot.h"
#include "pose.h"

void Log(const char *t);

bool cmdStub(JsonObject&  j)
{
	return false;
}

bool cmdTest1(JsonObject&  j)
{
	Log("robot1::EscEnable");
	Log("cmd_Test1");
	Serial.print("//cmd_Test1\n");

	return true;
}

bool cmdReset(JsonObject&  j)
{
	// by including specific variables, you can set pose to a particular value
	Log("robot1::ResetPose");
	M1.Reset();
	M2.Reset();
	X = Y = H = previousHeading = 0.0;
	if (j.containsKey("X"))
		X = j["X"];
	if (j.containsKey("Y"))
		X = j["Y"];
	if (j.containsKey("Z"))
		X = j["Z"];
	if (j.containsKey("H"))
	{
		Serial.print("// heading\n");
		X = j["H"];
		previousHeading = ypr[0];	// base value
	}
		
	return true;
}

bool cmdEsc(JsonObject&  j)
{
	Log("robot1::EscEnable");
	if (strcmp(j["Value"], "1") == 0)
	{
		escEnabled = true;
		digitalWrite(12, true);	// +++ hardcoded pin
	}
	if (strcmp(j["Value"], "0") == 0)
	{
		escEnabled = false;
		digitalWrite(12, false); // +++ hardcoded pin
	}

	return true;
}

bool cmdGeom(JsonObject&  j)
{
	Log("robot1::EscEnable");
	return false;
}

bool cmdMove(JsonObject&  j)
{
	Log("robot1::EscEnable");
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

////////////////////////////////////////////////////

void SetPower(PilotMotor& m, int p)
{
	Log("robot1::EscEnable");
	if (escEnabled)
	{
		char t[32];
		if (m.pwmPin != -1)
		{
			p = constrain(p, -100, 100);
			if (p != m.lastPower) // only set if changed
			{
				sprintf(t, "//Set Power %d\n", p); Serial.print(t);
				m.SetSpeed(p);	// +++ stub
			}
		}
	}
}


