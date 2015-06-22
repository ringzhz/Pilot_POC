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
	// !!! seems like about 3-4 floats is all the arduino (serial) can handle
	// often carries some debugging payload with it
	StaticJsonBuffer<128> publishBuffer;
	JsonObject& root = publishBuffer.createObject();
	root["T"] = "Heartbeat"; 
	root["T1"].set(M1.tgtVelocity, 2);  // number of decimals to print
	root["V1"].set(M1.velocity, 2);
	root["I1"].set(M1.integral, 2);
	root["D1"].set(M1.derivative, 2);
	root["PW1"].set(M1.lastPinPower, 2);

	root["T2"].set(M2.tgtVelocity, 2);  // number of decimals to print
	root["V2"].set(M2.velocity, 2);
	root["I2"].set(M2.integral, 2);
	root["D2"].set(M2.derivative, 2);
	root["PW2"].set(M2.lastPinPower, 2);

	// used for ticks per meter calibration
	//	root["TA1"] = M1.GetRawTacho();

	root.printTo(Serial); Serial.println();
}

void BumperEvent(bool bumperPressed)
{
	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Bumper";
	root["Value"] = bumperPressed ? 1 : 0;
	root.printTo(Serial); Serial.println();
}

// +++ log really needs to help with things like this;
// //DBGP("PinPower");  DBGV("Pin", pwmPin);  DBGV("", p); DBGE();
// once MPU code is separated out, maybe we will have enough room to include sprintf

void Log(char const *t)
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Log";
	root["Msg"] = t;
	root.printTo(Serial); Serial.println();
}

void MoveCompleteEvent(bool success)
{
	extern bool headingStop;
	extern bool moveStop;

	StaticJsonBuffer<64> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Moved";
	root["Value"] = success ? 1 : 0;
	root.printTo(Serial); Serial.println();
}

