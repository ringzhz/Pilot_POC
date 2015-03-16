#import <Arduino.h>
#include "PilotMotor.h"

volatile byte previousPins;
volatile long tacho[2];		// interrupt 0 and interrupt 1 tachos

ISR(MotorISR)
{
	byte intr, b, b_pin, idx;
	byte changedPins = PINB ^ previousPins;
	previousPins = PINB; // Save the previous state so you can tell what changed

	if (changedPins & (1 << PCINT0))
	{
		intr = 2;
		b_pin = 8;
		idx = 0;
	}

	if (changedPins & (1 << PCINT1))
	{
		intr = 3;
		b_pin = 9;
		idx = 1;
	}

	// else cause firey explosion

	b = digitalRead(b_pin);
	if (digitalRead(intr))
		b ? tacho[idx]++ : tacho[idx]--;
	else
		b ? tacho[idx]++ : tacho[idx]--;

}

void MotorInit()
{
	// hardcoded
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);

	previousPins = PINB & (PCINT0 | PCINT1);	// save current state
	attachInterrupt(PCINT0, MotorISR, CHANGE);
	attachInterrupt(PCINT1, MotorISR, CHANGE);
}

PilotMotor::PilotMotor(int pwm, int dir, int idx, bool revrsd)
{
	pwmPin = pwm;
	dirPin = dir;
	interruptIndex = idx;
	reverse = revrsd;
	lastTacho = 0L;
	power = 0;
	motorCW = true;

	pinMode(pwm, OUTPUT);
	pinMode(dir, OUTPUT);
	tacho[interruptIndex] = 0L;
}

long PilotMotor::GetTacho()
{
	return tacho[interruptIndex];
}

void PilotMotor::Tick()
{

}