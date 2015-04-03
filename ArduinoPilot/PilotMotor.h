//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#ifndef __PilotMotor__H
#define __PilotMotor__H

class PilotMotor
{
public:
	char motorName[8];
	int pwmPin;
	int dirPin;
	int feedBackPin;
	int interruptIndex;
	bool reversed;

	long lastUpdateTime;
	long lastTacho;
	float desiredSpeed;
	float actualSpeed;

	float lastPower;
	float previousError;
	float previousIntegral;

	PilotMotor(const char *n, int pwm, int dir, int fb, int idx, bool revrsd);
	void Reset();
	long GetTacho();
	void SetSpeed(int spd);
	void Tick();
};

void MotorInit();

#endif