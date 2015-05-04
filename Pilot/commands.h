//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#include "digitalWriteFast.h"

struct CmdFunction
{
	const char *cmd;
	void (*f)(JsonObject&  j);
};

void cmdTest1(JsonObject&  j)
{
	Serial.println("//Test1");
}

void cmdTest2(JsonObject&  j)
{
	Serial.println("//Test2");
	M1.SetSpeed(100, 0, NOLIMIT);
	//M2.Stop(false);
}

//////////////////////////////////////////////////

void cmdPid(JsonObject&  j)
{
	char * pKey = "P";
	char * iKey = "I";
	char * dKey = "D";
	int idx = -1;
	if (j.containsKey("Idx"))
	{
		int idx = j["Idx"];
		if (idx == 0 || idx == 1)
		{
			if (j.containsKey(pKey))
				PidTable[idx].Kp = j[pKey].as<float>();
			if (j.containsKey(iKey))
				PidTable[idx].Ki = j[iKey].as<float>();
			if (j.containsKey(dKey))
				PidTable[idx].Kd = j[dKey].as<float>();
#if 1
			Serial.print("// pid idx="); Serial.println(idx);
			Serial.print("//  p="); Serial.println(PidTable[idx].Kp);
			Serial.print("//  i="); Serial.println(PidTable[idx].Ki);
			Serial.print("//  d="); Serial.println(PidTable[idx].Kd);
#endif
		}
	}
}

void cmdBump(JsonObject&  j)
{
	BumperEventEnabled = j[value] == 1;
}

void cmdDest(JsonObject&  j)
{
	DestinationEventEnabled = j[value] == 1;
}

void cmdHeartbeat(JsonObject&  j)
{
	heartbeatEventEnabled = j[value] == 1;
	if (j.containsKey(intvl))
		heartbeatEventFrequency = j[intvl];
}

void cmdReset(JsonObject&  j)
{
	// by including specific variables, you can set pose to a particular value
	char * xKey = "X";
	char * yKey = "Y";
	char * hKey = "H";
	M1.Reset();
	M2.Reset();
	X = Y = H = 0.0;
	if (j.containsKey(xKey))
		X = j[xKey];
	if (j.containsKey(yKey))
		Y = j[yKey];
	if (j.containsKey(hKey))
		H = DEG_TO_RAD * (float)j[hKey];

	previousYaw = H + ypr[0];	// base value
}

void cmdEsc(JsonObject&  j)
{
	escEnabled = j[value] == 1;
	digitalWriteFast(ESC_ENA, escEnabled);
}

void cmdPose(JsonObject&  j)
{
	PoseEventEnabled = j[value] == 1;
	if (j.containsKey(intvl))
		CalcPoseFrequency = j[intvl];
}

void cmdGeom(JsonObject&  j)
{

}

// a passthrough TDD function
void cmdMotor(JsonObject&  j)
{
	if (escEnabled)
	{
		char * M1key = "1";
		char * M2key = "2";
		if (j.containsKey(M1key))
		{
			int s = (int) j[M1key];
			M1.SetSpeed(s, 0, s >= 0 ? +NOLIMIT : -NOLIMIT);	// +++acceleration(and pid) not working
		}
		if (j.containsKey(M2key))
		{
			int s = (int) j[M2key];
			M2.SetSpeed(s, 0, s >= 0 ? +NOLIMIT : -NOLIMIT);
		}
	}
}

void cmdMove(JsonObject&  j)
{
}

void cmdRot(JsonObject&  j)
{	
	const char *abs = "Abs";
	const char *rel = "Rel";
	float headingGoal = H;
	if (j.containsKey(abs))
		headingGoal = (float) j[abs];
	else if (j.containsKey(rel))
		headingGoal = H + (float) j[rel];
}

void cmdGoto(JsonObject&  j)
{
}

////////////////////////////////////////////////////

CmdFunction cmdTable[] {
	{ "Test1", cmdTest1 },
	{ "Test2", cmdTest2 },
	{ "Reset", cmdReset },
	{ "Geom", cmdGeom },
	{ "PID", cmdPid },
	{ "Esc", cmdEsc },
	{ "Rot", cmdRot, },
	{ "GoTo", cmdGoto, },
	{ "Move", cmdMove },
	{ "Bumper", cmdBump, },
	{ "Heartbeat", cmdHeartbeat, },
	{ "Pose", cmdPose, },
	{ "M", cmdMotor, },
};

void ProcessCommand(JsonObject& j)
{
	for (int i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
		if (strcmp(cmdTable[i].cmd, (const char *) j["Cmd"]) == 0)
			(*cmdTable[i].f)(j);
}

