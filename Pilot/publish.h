//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

void PublishPose()
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Pose";
	root["X"].set(X / 1000, 4);		// mm to meter
	root["Y"].set(Y / 1000, 4);
	root["H"].set(RAD_TO_DEG * H, 1);	// in degrees
	root.printTo(Serial); Serial.println();
}

void PublishHeartbeat()
{
	// often carries some debugging payload with it
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Heartbeat";

// used for PID tuning
#if 1
	root["T1"].set(M1.tgtVelocity, 2);  // number of decimals to print
	root["T2"].set(M2.tgtVelocity, 2);
	root["V1"].set(M1.velocity, 2);
	root["V2"].set(M2.velocity, 2);
#endif
#if 0
	root["P1"].set(M1.lastPinPower, 2);
	root["P2"].set(M2.lastPinPower, 2);
#endif
	// used for ticks per meter calibration
#if 1
	root["TA1"] = M1.GetRawTacho();
	root["TA2"] = M2.GetRawTacho();
#endif
	root.printTo(Serial); Serial.println();
}

void BumperEvent(bool bumperPressed)
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Bumper";
	root[value] = bumperPressed ? 1 : 0;
	root.printTo(Serial); Serial.println();
}

void MoveCompleteEvent(bool success)
{
	extern bool headingStop;
	extern bool moveStop;

	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	headingStop = moveStop = false;
	root["T"] = "Moved";
	root[value] = success ? 1 : 0;
	root.printTo(Serial); Serial.println();
}

