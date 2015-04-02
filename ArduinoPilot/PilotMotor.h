#ifndef __PilotMotor__H
#define __PilotMotor__H

#include <Stream.h>

/*
	reality is the atmel is pretty dumb. It does not know what pin caused
	an interrupt (brilliant). So to do anything object oriented, 
	we really need to fake it.

	a hardcoded array is set to phaseB pins so the interrupt routine can 
	read them after determining what pin has changed. The tacho counts are stored
	in a hardcoded array, and the PilotMotor instance just uses this array.
	The PilotMotor does not need to know what pins they use in this case, so they
	use the array index instead. what an f'n kludge.

*/


class PilotMotor
{
public:
	char id[16];
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