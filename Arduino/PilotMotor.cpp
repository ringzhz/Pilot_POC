#import <Arduino.h>
#include "PilotMotor.h"

volatile long tacho[2];		// interrupt 0 and interrupt 1 tachos

ISR(MotorISR1)
{
	// +++ fastRead ????
	// byte a = digitalPinToPort(2) & digitalPinToBitMask(8);
	// byte b = digitalPinToPort(8) & digitalPinToBitMask(8);

	int b = digitalRead(8);

	if (digitalRead(2))
		b ? tacho[0]++ : tacho[0]--;
	else
		b ? tacho[0]-- : tacho[0]++;
}

ISR(MotorISR2)
{
	int b = digitalRead(9);

	if (digitalRead(3))
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

PilotMotor::PilotMotor(const char *n, Stream& dbg, int pwm, int dir, int fb, int idx, bool revrsd)
{
	strncpy(id, n, sizeof(id)-1);
	db = &dbg;
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
	sprintf(t, "new power %s %d\n", id, newSpeed); db->print(t);
}

void PilotMotor::Tick()
{

}