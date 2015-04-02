#include <Arduino.h>
#include <digitalWriteFast.h>
#include "PilotMotor.h"

volatile long tacho[2];		// interrupt 0 and interrupt 1 tachos

ISR(MotorISR1)
{
	// +++ fastRead ????

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
	// hardcoded interrupt handlers
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);

	attachInterrupt(PCINT0, MotorISR1, CHANGE);	// pin 2
	attachInterrupt(PCINT1, MotorISR2, CHANGE); // pin 3
}

PilotMotor::PilotMotor(const char *n, int pwm, int dir, int fb, int idx, bool revrsd)
{
	strncpy(id, n, sizeof(id)-1);
	pwmPin = pwm;
	dirPin = dir;
	feedBackPin = fb;
	interruptIndex = idx;
	reversed = revrsd;
	lastTacho = 0L;
	lastPower = 0;

	if (pwm != -1)
	{
		pinMode(pwm, OUTPUT);
		pinMode(dir, OUTPUT);
	}

	tacho[interruptIndex] = 0L;
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
	char t[32];
	// speed is a +/- percent of max
	int newSpeed = constrain(spd, -100, 100);
	digitalWrite(dirPin, (newSpeed >= 0) * reversed);
	analogWrite(pwmPin, map(abs(newSpeed), 0, 100, 0, 255));
	desiredSpeed = newSpeed;
	sprintf(t, "new power %s %d\n", id, newSpeed); Serial.print(t);
}

void PilotMotor::Tick()
{

}