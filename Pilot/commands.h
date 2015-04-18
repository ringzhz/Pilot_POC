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
	DBGLOG("::cmdTest1");
	Serial.print("// cmdTest1\r\n");
	return true;
}

//////////////////////////////////////////////////

bool cmdMmax(JsonObject&  j)
{
	DBGLOG("::cmdMmax");
	MotorMax = j["Value"];
	return true;
}

bool cmdPid1(JsonObject&  j)
{
	DBGLOG("::cmdPid1");
	Kp1 = j["P"];
	Ki1 = j["I"];
	Kd1 = j["D"];
	return true;
}

bool cmdBump(JsonObject&  j)
{
	DBGLOG("::cmdBump");
	BumperEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdServo(JsonObject&  j)
{
	DBGLOG("::cmdServo");
	return true;
}

bool cmdDest(JsonObject&  j)
{
	DBGLOG("::cmdDest");
	DestinationEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdHeartbeat(JsonObject&  j)
{
	DBGLOG("::cmdHeartbeat");
	heartbeatEventEnabled = j["Value"] == 1;
	if (j.containsKey("Int"))
		heartbeatEventFrequency = j["Int"];
	return true;
}

bool cmdPing(JsonObject&  j)
{
	DBGLOG("::cmdPing");
	pingEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdReset(JsonObject&  j)
{
	// by including specific variables, you can set pose to a particular value
	DBGLOG("::cmdReset");
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
	DBGLOG("::cmdEsc");
	escEnabled = j["Value"] == 1;
	digitalWriteFast(ESC_ENA, escEnabled);
	return true;
}

bool cmdPose(JsonObject&  j)
{
	DBGLOG("::cmdPose");
	PoseEventEnabled = j["Value"] == 1;
	if (j.containsKey("Int"))
		CalcPoseFrequency = j["Int"];
	return true;
}

bool cmdGeom(JsonObject&  j)
{
	// +++
	DBGLOG("::cmdGeom");
	return false;
}

bool cmdPower(JsonObject&  j)
{
	// +++ actually more of a testing function, will probably go away
	//char t[16];
	DBGLOG("::cmdPower");

	if (escEnabled)
	{
		int p = constrain((int)j["Value"], -100, 100);
		//sprintf(t, "// P %d\n", p); Serial.print(t);
		M1.SetSpeed(p);
		M2.SetSpeed(p);
	}

	return true;
}

bool cmdMove(JsonObject&  j)
{
	DBGLOG("::cmdMove");
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

bool cmdRot(JsonObject&  j)
{
	DBGLOG("::cmdRot");
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

bool cmdGoto(JsonObject&  j)
{
	DBGLOG("::cmdGoto");
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
	{ "Bump", cmdBump, },
	{ "Servo", cmdServo, },
	{ "Heartbeat", cmdHeartbeat, },
	{ "Pose", cmdPose, },
	{ "Power", cmdPower, },
	{ "Ping", cmdPing, },
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

