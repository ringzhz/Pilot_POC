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

uint8_t MotorMax = 100;
float Kp1, Ki1, Kd1;

volatile long tacho[2];		// interrupt 0 & 1 tachometers

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

PilotMotor::PilotMotor(const char *name, uint8_t pwm, uint8_t dir, uint8_t fb, uint8_t idx, bool revrsd)
{
	strncpy(motorName, name, sizeof(motorName));
	pwmPin = pwm;
	dirPin = dir;
	feedBackPin = fb;
	interruptIndex = idx;
	reversed = revrsd;

	if (escEnabled)
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

uint32_t PilotMotor::GetTacho()
{
	return reversed ? -tacho[interruptIndex] : tacho[interruptIndex];
}

void PilotMotor::SetSpeed(int spd)
{
	char t[64];
	// speed is a +/- percent of max	
	int newSpeed = map(spd, -100, 100, -MotorMax, MotorMax);
	newSpeed = constrain(newSpeed, -100, 100);
	digitalWrite(dirPin, (newSpeed >= 0) * reversed);
	analogWrite(pwmPin, map(abs(newSpeed), 0, 100, 0, 255));
	desiredSpeed = newSpeed;
	sprintf(t, "// %d >> %s\n", newSpeed, motorName); Serial.print(t);
}

///////////////////////////////////////////////////

void MotorInit()
{
	Serial.print("// MotorInit ... \n");

	tacho[0] = tacho[1] = 0L;

	// !! hardcoded interrupt handlers !!
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);

	attachInterrupt(PCINT0, MotorISR1, CHANGE);	// pin 2
	attachInterrupt(PCINT1, MotorISR2, CHANGE); // pin 3
}



void PilotMotorTick()
{
	// +++ regulation needed now! but waiting on hardware :|
}