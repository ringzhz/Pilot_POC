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

bool cmd_Reset(JsonObject& j);
bool cmd_Esc(JsonObject&  j);
bool cmd_Geom(JsonObject&  j);
bool cmd_Move(JsonObject&  j);
bool cmd_GPS(JsonObject&  j);
bool cmd_Test1(JsonObject&  j);
bool cmd_Test2(JsonObject&  j);

#endif
