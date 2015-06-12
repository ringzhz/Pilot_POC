//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

struct CmdFunction
{
	const char *cmd;
	void (*f)(JsonObject&  j);
};

void cmdVars(JsonObject&  j)
{
	char *vKey = "Vals";
	// +++ motor reverses, other globals TBD, merge with geom???
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
	extern void PublishPose();
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
	
	NormalizeHeading(H, -PI, PI);
	previousYaw = H + ypr[0];	// base value
	//Traveling = Rotating = false;
	PublishPose();
}

void cmdEsc(JsonObject&  j)
{
	escEnabled = j[value] == 1;
	digitalWriteFast(ESC_ENA, escEnabled);
}

void cmdPose(JsonObject&  j)
{
	PoseEventEnabled = j[value] == 1;
}

void cmdConfig(JsonObject&  j)
{
	extern MPU6050 mpu;
	extern pidData MotorPID;

	// combines old PID, geom, and calibrate, adds some new stuff
	char *geomKey = "Geom";
	char *pidKey = "PID";
	char *mpuKey = "MPU";

	if (j.containsKey(geomKey))
	{
		Geom.ticksPerMeter = j[geomKey][0].as<float>();;
		Geom.mMax = j[geomKey][1].as<float>();;
	}

	if (j.containsKey(pidKey))
	{
		MotorPID.Kp = j[pidKey][0].as<float>();
		MotorPID.Ki = j[pidKey][1].as<float>();
		MotorPID.Kd = j[pidKey][2].as<float>();
		M1.previousError = M2.previousError = M1.integral = M2.integral = M1.derivative = M2.derivative = 0;
	}

	// MPU/DMP calibration values
	//DBGP("Cali"); DBGV("accX", (int)j[vKey][0]); DBGE();
	if (j.containsKey(mpuKey))
	{
		mpu.setXAccelOffset(j[mpuKey][0]);
		mpu.setYAccelOffset(j[mpuKey][1]);
		mpu.setZAccelOffset(j[mpuKey][2]);
		mpu.setXGyroOffset(j[mpuKey][3]);
		mpu.setYGyroOffset(j[mpuKey][4]);
		mpu.setZGyroOffset(j[mpuKey][5]);
	}
}

void cmdPower(JsonObject&  j)
{
	extern bool headingStop;
	extern float headingGoal;

	int acc = 0;				// +++acceleration not implemented
	char *m1Key = "M1";
	char *m2Key = "M2";
	char *headingStopKey = "hStop";
	char *distStopKey = "dStop";
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
	if (j.containsKey(headingStopKey))
	{
		float g = j[headingStopKey].as<float>() * DEG_TO_RAD;
		NormalizeHeading(g, -PI, PI);
		headingGoal = g;
		headingStop = true;
	}
	if (j.containsKey(distStopKey))
	{
		// ++++
	}
}

////////////////////////////////////////////////////

CmdFunction cmdTable[] {
	{ "Esc", cmdEsc },
	{ "Pwr", cmdPower, },
	{ "Reset", cmdReset },
	{ "Bumper", cmdBump, },
	{ "Heartbeat", cmdHeartbeat, },
	{ "Pose", cmdPose, },
	{ "Config", cmdConfig, },
};

void ProcessCommand(JsonObject& j)
{
	for (int i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
		if (strcmp(cmdTable[i].cmd, j["Cmd"]) == 0)
		{
			(*cmdTable[i].f)(j);
			break;
		}
}

