//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright (c) 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include "digitalWriteFast.h"

struct CmdFunction
{
	const char *cmd;
	bool(*f)(JsonObject&  j);
};

bool cmdTest1(JsonObject&  j)
{
	Serial.print(F("// cmdTest1\r\n"));
	return true;
}

//////////////////////////////////////////////////

bool cmdMmax(JsonObject&  j)
{
	MotorMax = j["Value"];
	return true;
}

bool cmdPid1(JsonObject&  j)
{
	Kp1 = j["P"];
	Ki1 = j["I"];
	Kd1 = j["D"];
	return true;
}

bool cmdPid2(JsonObject&  j)
{
	//Kp1 = j["P"];
	//Ki1 = j["I"];
	//Kd1 = j["D"];
	//return true;
	return false;
}

bool cmdBump(JsonObject&  j)
{
	BumperEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdServo(JsonObject&  j)
{
	return true;
}

bool cmdDest(JsonObject&  j)
{
	DestinationEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdHeartbeat(JsonObject&  j)
{
	heartbeatEventEnabled = j["Value"] == 1;
	if (j.containsKey("Int"))
		heartbeatEventFrequency = j["Int"];
	return true;
}

bool cmdReset(JsonObject&  j)
{
	// by including specific variables, you can set pose to a particular value
	M1.Reset();
	M2.Reset();
	X = Y = H = previousYaw = 0.0;
	if (j.containsKey("X"))
		X = j["X"];
	if (j.containsKey("Y"))
		Y = j["Y"];
	if (j.containsKey("H"))
		H = DEG_TO_RAD * (float)j["H"];

	previousYaw = ypr[0];	// base value

	return true;
}

bool cmdEsc(JsonObject&  j)
{
	escEnabled = j["Value"] == 1;
	digitalWriteFast(ESC_ENA, escEnabled);
	return true;
}

bool cmdPose(JsonObject&  j)
{
	PoseEventEnabled = j["Value"] == 1;
	if (j.containsKey("Int"))
		CalcPoseFrequency = j["Int"];
	return true;
}

bool cmdGeom(JsonObject&  j)
{
	// +++
	Serial.print(F("//! cmdGeom"));
	return false;
}

bool cmdPower(JsonObject&  j)
{
	// +++ actually more of a testing function, will probably go away
	//char t[16];

	if (escEnabled)
	{
		int p = constrain((int)j["Value"], -100, 100);
		//sprintf(t, "// P %d\n", p); Serial.print(t);
		M1.SetPower(p);
		M2.SetPower(p);
	}
	return true;
}

bool cmdMove(JsonObject&  j)
{
	float speed = (int)j["Speed"] / 10.0F;
	return false;
}

bool cmdRot(JsonObject&  j)
{
	float speed = (int)j["Speed"] / 10.0F;
	return false;
}

bool cmdGoto(JsonObject&  j)
{
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
	{ "PID2", cmdPid2 },
	{ "Esc", cmdEsc },
	{ "Rot", cmdRot, },
	{ "GoTo", cmdGoto, },
	{ "Move", cmdMove },
	{ "Bumper", cmdBump, },
	{ "Servo", cmdServo, },
	{ "Heartbeat", cmdHeartbeat, },
	{ "Pose", cmdPose, },
	{ "Power", cmdPower, },
};

bool ProcessCommand(JsonObject& j)
{
	bool rc = false;
	for (int i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
		if (strcmp(cmdTable[i].cmd, (const char *)j["Cmd"]) == 0)
		{
			rc = (*cmdTable[i].f)(j);
			break;
		}
	return rc;
}

