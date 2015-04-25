//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#include "digitalWriteFast.h"

struct CmdFunction
{
	const char *cmd;
	bool(*f)(JsonObject&  j);
};

bool cmdTest1(JsonObject&  j)
{
	Serial.print("// cmdTest1\n");
	M1.SetSpeed(50, ACCELERATION, +NOLIMIT);
	return true;
}

bool cmdTest2(JsonObject&  j)
{
	Serial.print("// cmdTest1\n");
	M1.Stop(false);
#if 0
	Serial.print("// cmdTest2 ");
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	JsonObject& geom = root.createNestedObject("Geom");
	geom["tpr"].set(Geom.ticksPerRevolution, 4);
	geom["wd"].set(Geom.wheelDiameter, 2);
	geom["wb"].set(Geom.wheelBase, 2);
	geom["es"].set(Geom.EncoderScaler, 4);
	root.printTo(Serial);
	Serial.print("\n");
#endif
	return true;
}

//////////////////////////////////////////////////

bool cmdPid1(JsonObject&  j)
{
	Kp1 = j["P"];
	Ki1 = j["I"];
	Kd1 = j["D"];
	return true;
}

bool cmdBump(JsonObject&  j)
{
	BumperEventEnabled = j[value] == 1;
	return true;
}

bool cmdDest(JsonObject&  j)
{
	DestinationEventEnabled = j[value] == 1;
	return true;
}

bool cmdHeartbeat(JsonObject&  j)
{
	heartbeatEventEnabled = j[value] == 1;
	if (j.containsKey(intvl))
		heartbeatEventFrequency = j[intvl];
	return true;
}

bool cmdReset(JsonObject&  j)
{
	// by including specific variables, you can set pose to a particular value
	M1.Reset();
	M2.Reset();
	X = Y = H = 0.0;
	if (j.containsKey("X"))
		X = j["X"];
	if (j.containsKey("Y"))
		Y = j["Y"];
	if (j.containsKey("H"))
		H = DEG_TO_RAD * (float)j["H"];

	previousYaw = H + ypr[0];	// base value
	return true;
}

bool cmdEsc(JsonObject&  j)
{
	escEnabled = j[value] == 1;
	digitalWriteFast(ESC_ENA, escEnabled);
	return true;
}

bool cmdPose(JsonObject&  j)
{
	PoseEventEnabled = j[value] == 1;
	if (j.containsKey(intvl))
		CalcPoseFrequency = j[intvl];
	return true;
}

bool cmdGeom(JsonObject&  j)
{
	// +++
	Serial.print("//! cmdGeom");
	return false;
}

// a passthrough TDD function
bool cmdMotor(JsonObject&  j)
{
	if (escEnabled)
	{
		const char * M1key = "1";
		const char * M2key = "2";
		if (j.containsKey(M1key))
		{
			int s = (int) j[M1key];
			M1.SetSpeed(abs(s), ACCELERATION, s >= 0 ? +NOLIMIT : -NOLIMIT);
		}
		if (j.containsKey(M2key))
		{
			int s = (int) j[M2key];
			M2.SetSpeed(abs(s), ACCELERATION, s >= 0 ? +NOLIMIT : -NOLIMIT);
		}
	}
	return true;
}

bool cmdMove(JsonObject&  j)
{
	float dist = (int)j["Dist"] / 1000;	// meter to mm
	StartMove(X - sin(H) * dist, Y + cos(H) * dist);	// straight along current heading, H0 is north
	return true;
}

bool cmdRot(JsonObject&  j)
{	
	const char *abs = "Abs";
	const char *rel = "Rel";
	float headingGoal = H;
	if (j.containsKey(abs))
		headingGoal = (float) j[abs];
	else if (j.containsKey(rel))
		headingGoal = H + (float) j[rel];
	StartRotate(headingGoal);
}

bool cmdGoto(JsonObject&  j)
{
	StartMove((float) j["X"], (float) j["Y"]);
	return true;
}

////////////////////////////////////////////////////

CmdFunction cmdTable[] {
	{ "Test1", cmdTest1 },
	{ "Test2", cmdTest2 },
	{ "Reset", cmdReset },
	{ "Geom", cmdGeom },
	{ "PID1", cmdPid1 },
	{ "Esc", cmdEsc },
	{ "Rot", cmdRot, },
	{ "GoTo", cmdGoto, },
	{ "Move", cmdMove },
	{ "Bumper", cmdBump, },
	{ "Heartbeat", cmdHeartbeat, },
	{ "Pose", cmdPose, },
	{ "M", cmdMotor, },
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

