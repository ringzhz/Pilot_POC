//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <digitalWriteFast.h>
#include <ArduinoJson.h>

#include "MPU6050.h"
#include "helper_3dmath.h"
#include "PilotMotor.h"
#include "ArduinoPilot.h"
#include "pose.h"
#include "Commands.h"

//////////////////////////////////////////////////


//////////////////////////////////////////////////

bool cmdStub(JsonObject&  j)
{
	Log(F("::cmdStub"));
	return false;
}

bool cmdTest1(JsonObject&  j)
{
	Log(F("::cmdTest1"));
	Serial.print(F("// cmdTest1\n"));
	return true;
}

bool cmdMmax(JsonObject&  j)
{
	Log(F("::cmdMmax"));
	MotorMax = j["Value"];
	return true;
}

bool cmdPid1(JsonObject&  j)
{
	Log(F("::cmdPid1"));
	Kp = j["P"];
	Ki = j["I"];
	Kd = j["D"];
	return true;
}

bool cmdBump(JsonObject&  j)
{
	Log(F("::cmdBump"));
	BumperEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdDest(JsonObject&  j)
{
	Log(F("::cmdDest"));
	DestinationEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdHeartbeat(JsonObject&  j)
{
	Log(F("::cmdHeartbeat"));
	heartbeatEventEnabled = j["Value"] == 1;
	if (j.containsKey("Int"))
		heartbeatEventFrequency = j["Int"];
	return true;
}

bool cmdPing(JsonObject&  j)
{
	Log(F("::cmdPing"));
	pingEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdReset(JsonObject&  j)
{
	// by including specific variables, you can set pose to a particular value
	Log(F("::cmdReset"));
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
		X = j["H"];
		previousHeading = ypr[0];	// base value
	}

	return true;
}

bool cmdEsc(JsonObject&  j)
{
	Log(F("::cmdEsc"));
	escEnabled = j["Value"] == 1;
	digitalWriteFast(ESC_EN, escEnabled);
	return true;
}

bool cmdGeom(JsonObject&  j)
{
	Log(F("::cmdGeom"));
	return false;
}

bool cmdPower(JsonObject&  j)
{
	char t[16];
	// +++ actually more of a testing function, will probably go away
	Log(F("::cmdPower"));
	
	if (escEnabled)
	{
		int p = constrain(j["Power"], -100, 100);
		sprintf_P(t, "// P %d\n", p); Serial.print(t);
		M1.SetSpeed(p);
		M2.SetSpeed(p);
	}

	return true;
}

bool cmdMove(JsonObject&  j)
{
	Log(F("::cmdMove"));
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

bool cmdRot(JsonObject&  j)
{
	Log(F("::cmdRot"));
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

bool cmdGoto(JsonObject&  j)
{
	Log(F("::cmdGoto"));
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

////////////////////////////////////////////////////

CmdFunction cmdTable[] {
	{ "Test1", cmdTest1 },
	{ "Reset", cmdReset },
	{ "Geom", cmdGeom },
	{ "MMax", cmdMmax },
	{ "PID1", cmdPid1 },
	{ "Esc", cmdEsc },
	{ "Rot", cmdRot, },
	{ "GoTo", cmdGoto, },
	{ "Move", cmdMove },
	{ "Bump", cmdBump, },
	{ "Dest", cmdDest, },
	{ "HB", cmdHeartbeat, },
	{ "Ping", cmdPing, },
};

void ProcessCommand(JsonObject& j)
{
	for (int i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
		if (strcmp(cmdTable[i].cmd, (const char *)j["Cmd"]) == 0)
		{
			bool rc = (*cmdTable[i].f)(j);
			break;
		}
}