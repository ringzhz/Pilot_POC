//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

struct CmdFunction
{
	const char *cmd;
	void (*f)(JsonObject&  j);
};

// MPU/DMP calibration values
void cmdCali(JsonObject&  j)
{
	extern MPU6050 mpu;
	char *vKey = "Vals";
	// warning, no error checking!!
	DBGP("Cali");
	DBGV("accX", (int)j[vKey][0]);
	mpu.setXAccelOffset(j[vKey][0]);
	mpu.setYAccelOffset(j[vKey][1]);
	mpu.setZAccelOffset(j[vKey][2]);
	mpu.setXGyroOffset(j[vKey][3]);
	mpu.setYGyroOffset(j[vKey][4]);
	mpu.setZGyroOffset(j[vKey][5]);

	DBGE();
}

// PIDs
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
			if (idx == 0)
				M1.previousIntegral = M2.previousIntegral = M1.previousDerivative = M2.previousDerivative = 0;
			else if (idx == 1)
				previousIntegral = previousDerivative = previousDerivative = 0;
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
	extern bool GeomReceived;
	char *tprKey = "TPR";
	char *diamKey = "Diam";
	char *baseKey = "Base";
	char *maxKey = "mMax";

	if (j.containsKey(tprKey))
		Geom.ticksPerRevolution = j[tprKey];
	if (j.containsKey(diamKey))
		Geom.wheelDiameter = j[diamKey];
	if (j.containsKey(baseKey))
		Geom.wheelBase = j[baseKey];
	if (j.containsKey(maxKey))
		Geom.MMax = j[maxKey];
	Geom.EncoderScaler = Geom.ticksPerRevolution / (PI * Geom.wheelDiameter);
	GeomReceived = true;
}

void cmdPower(JsonObject&  j)
{
	int acc = 0;	// +++acceleration not implemented
	char *m1Key = "M1";
	char *m2Key = "M2";
	char *accKey = "Acc";
	if (j.containsKey(accKey))
		acc = j[accKey];
	if (j.containsKey(m1Key))
	{
		float s = j[m1Key];
		M1.SetSpeed(s, acc, s >= 0 ? +NOLIMIT : -NOLIMIT);
	}
	if (j.containsKey(m2Key))
	{
		float s = j[m2Key];
		M2.SetSpeed(s, acc, s >= 0 ? +NOLIMIT : -NOLIMIT);
	}	
}

void cmdRot(JsonObject&  j)
{	
	char *abs = "Abs";
	char *rel = "Rel";
	float headingGoal = H;
	if (j.containsKey(abs))
		headingGoal = (float) j[abs];
	else if (j.containsKey(rel))
		headingGoal = H + (float) j[rel];
	Rotating = true;
}

void cmdTravel(JsonObject&  j)
{
	char *distKey = "Dist";
	char *spdKey = "Spd";
	char *xKey = "X";
	char *yKey = "Y";
	if (j.containsKey(distKey) && j.containsKey(spdKey))
		Travel(j[distKey], j[spdKey]);
	else if (j.containsKey(xKey) && j.containsKey(yKey) && j.containsKey(spdKey))
		Travel(j[xKey], j[yKey], j[spdKey]);
}

////////////////////////////////////////////////////

CmdFunction cmdTable[] {
	{ "Reset", cmdReset },
	{ "Geom", cmdGeom },
	{ "PID", cmdPid },
	{ "CALI", cmdCali },
	{ "Esc", cmdEsc },
	{ "Rot", cmdRot, },
	{ "Travel", cmdTravel, },
	{ "Bumper", cmdBump, },
	{ "Heartbeat", cmdHeartbeat, },
	{ "Pose", cmdPose, },
	{ "Pwr", cmdPower, },
};

void ProcessCommand(JsonObject& j)
{
	for (int i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
		if (strcmp(cmdTable[i].cmd, j["Cmd"]) == 0)
			(*cmdTable[i].f)(j);
}

