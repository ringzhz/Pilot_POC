//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright � 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <digitalWriteFast.h>
#include <ArduinoJson.h>

#include "MPU6050.h"
#include "helper_3dmath.h"
#include "PilotMotor.h"
#include "ArduinoPilot.h"
#include "pose.h"

volatile long tacho[2];		// interrupt 0 and interrupt 1 tachometers

ISR(MotorISR1)
{
	int b = digitalReadFast(8);
	if (digitalReadFast(2))
		b ? tacho[0]++ : tacho[0]--;
	else
		b ? tacho[0]-- : tacho[0]++;
}

ISR(MotorISR2)
{
	int b = digitalReadFast(9);
	if (digitalReadFast(3))
		b ? tacho[1]++ : tacho[1]--;
	else
		b ? tacho[1]-- : tacho[1]++;
}

void MotorInit()
{
	Serial.print("//MotorInit ... ");

	tacho[0] = 0L;
	tacho[1] = 0L;

	// hardcoded interrupt handlers
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);

	attachInterrupt(PCINT0, MotorISR1, CHANGE);	// pin 2
	attachInterrupt(PCINT1, MotorISR2, CHANGE); // pin 3
}

PilotMotor::PilotMotor(const char *name, int pwm, int dir, int fb, int idx, bool revrsd)
{
	strncpy(motorName, name, sizeof(motorName) - 1);
	pwmPin = pwm;
	dirPin = dir;
	feedBackPin = fb;
	interruptIndex = idx;
	reversed = revrsd;

	if (pwm != -1)
	{
		pinMode(pwm, OUTPUT);
		pinMode(dir, OUTPUT);
	}

	Reset();
}

void PilotMotor::Reset()
{
	tacho[interruptIndex] = lastTacho = 0L;
	lastPower = desiredSpeed = actualSpeed = 0.0;
	previousError = previousIntegral = 0.0;
	//lastUpdateTime = ? ? ? ;
}

long PilotMotor::GetTacho()
{
	return reversed ? -tacho[interruptIndex] : tacho[interruptIndex];
}

void PilotMotor::SetSpeed(int spd)
{
	// +++
	char t[32];
	// speed is a +/- percent of max
	int newSpeed = constrain(spd, -100, 100);
	digitalWrite(dirPin, (newSpeed >= 0) * reversed);
	analogWrite(pwmPin, map(abs(newSpeed), 0, 100, 0, 255));
	desiredSpeed = newSpeed;
	sprintf(t, "new power %s %d\n", motorName, newSpeed); Serial.print(t);
}

void PilotMotor::Tick()
{
	// +++ regulation needed now! but waiting on hardware :|
}