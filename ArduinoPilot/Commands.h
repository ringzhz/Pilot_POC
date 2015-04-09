//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#ifndef __commands_h
#define __commands_h

#include <ArduinoJson.h>

struct CmdFunction
{
	const char *cmd;
	bool(*f)(JsonObject&  j);
};

bool cmdStub(JsonObject& j);
bool cmdReset(JsonObject& j);
bool cmdEsc(JsonObject&  j);
bool cmdGeom(JsonObject&  j);
bool cmdMove(JsonObject&  j);
bool cmdTest1(JsonObject&  j);

#endif
