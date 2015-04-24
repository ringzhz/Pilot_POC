//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

void PublishPose()
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = robot1;
	root["T"] = "Pose";
	root["X"].set(X / 1000, 4);		// mm to meter
	root["Y"].set(Y / 1000, 4);
	root["H"].set(RAD_TO_DEG * H, 2);	// in degrees
	root.printTo(Serial); Serial.print(newline);
}

void PublishHeartbeat()
{
	// often carries some debugging payload with it
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = robot1;
	root["T"] = "Heartbeat";

#if 0
	root["M1Tach"].set(M1.GetTacho(), 0);  // 0 is the number of decimals to print
	root["M2Tach"].set(M2.GetTacho(), 0);
#endif
#if 0
	root["Yaw"].set(ypr[0], 4);
	root["Pit"].set(ypr[1], 4);
	root["Rol"].set(ypr[2], 4);
#endif
#if 1
	root["sp1"].set(M1.spTPM, 2);
	root["pv1"].set(M1.spTPM, 2);
	root["pe1"].set(M1.previousError, 2);
	root["sp2"].set(M2.spTPM, 2);
	root["pv2"].set(M2.spTPM, 2);
	root["pe2"].set(M2.previousError, 2);
#endif

	root.printTo(Serial); Serial.print(newline);
}

void BumperEvent(bool bumperPressed)
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = robot1;
	root["T"] = "Bumper";
	root[value] = bumperPressed ? 1 : 0;
	root.printTo(Serial); Serial.print(newline);
}

void MoveCompleteEvent(bool success)
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = robot1;
	root["T"] = "Complete";
	root[value] = success ? 1 : 0;
	root.printTo(Serial); Serial.print(newline);
}

