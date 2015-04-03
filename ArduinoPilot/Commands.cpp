//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include <Arduino.h>
#include "Commands.h"
#include "PilotMotor.h"

extern PilotMotor M1, M2;
extern double X, Y, H;
extern bool esc_enabled, gps_enabled;
void Log(const char *t);

bool cmd_Reset(JsonObject&  j)
{
	Log("robot1::ResetPose");
	Serial.print("//ResetPose\n");
	M1.Reset();
	M2.Reset();
	X = Y = H = 0.0;
	return true;
}

bool cmd_Esc(JsonObject&  j)
{
	return false;
}

bool cmd_Geom(JsonObject&  j)
{
	return false;
}

bool cmd_Move(JsonObject&  j)
{
	return false;
}

bool cmd_GPS(JsonObject&  j)
{
	return false;
}

bool cmd_Test1(JsonObject&  j)
{
	Log("cmd_Test1");
	Serial.print("//cmd_Test1\n");
	return true;
}

bool cmd_Test2(JsonObject&  j)
{
	return false;
}


void SetPower(PilotMotor& m, int p)
{
	if (esc_enabled)
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


