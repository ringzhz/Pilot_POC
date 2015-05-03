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
	root.printTo(Serial); Serial.print(newline);
}

void PublishHeartbeat()
{
	// often carries some debugging payload with it
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Heartbeat";

#if 1
	root["M1 vel"].set(M1.velocity, 2);  // 0 is the number of decimals to print
	root["M2 vel"].set(M2.velocity, 2);
#endif

	root.printTo(Serial); Serial.print(newline);
}

void BumperEvent(bool bumperPressed)
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Bumper";
	root[value] = bumperPressed ? 1 : 0;
	root.printTo(Serial); Serial.print(newline);
}

void MoveCompleteEvent(bool success)
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Complete";
	root[value] = success ? 1 : 0;
	root.printTo(Serial); Serial.print(newline);
}

