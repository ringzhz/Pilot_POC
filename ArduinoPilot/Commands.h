//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include <ArduinoJson.h>

struct CmdFunction
{
	const char *cmd;
	bool(*f)(JsonObject&  j);
};

extern CmdFunction cmdTable[];
void ProcessCommand(JsonObject& j);
