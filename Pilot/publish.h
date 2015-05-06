	//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

void PublishPose()
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Pose";
	root["X"].set(X / 1000, 4);		// mm to meter
	root["Y"].set(Y / 1000, 4);
	root["H"].set(RAD_TO_DEG * H, 2);	// in degrees
	root.printTo(Serial); Serial.println();
}

void PublishHeartbeat()
{
	// often carries some debugging payload with it
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Heartbeat";

#if 1
	root["V1"].set(M1.velocity, 2);  // 2 is the number of decimals to print
	root["V2"].set(M2.velocity, 2);
	root["T1"] = M1.GetRawTacho(); 
	root["T2"] = M2.GetRawTacho();
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
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Moved";
	root[value] = success ? 1 : 0;
	root.printTo(Serial); Serial.println();
}

