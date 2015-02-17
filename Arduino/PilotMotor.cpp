#import <Arduino.h>
#include "PilotMotor.h"

volatile byte previousPins;
volatile long tacho[2];		// interrupt 0 and interrupt 1 tachos

ISR(MotorISR)
{
	int intr, ph_b, t_idx;
	byte changedPins = PINB ^ previousPins;
	previousPins = PINB; // Save the previous state so you can tell what changed

	// hope there is no collision :(
	if (changedPins & (1 << PCINT0))
	{
		intr = 2;
		ph_b = 8;
		t_idx = 0;
	}
	if (changedPins & (1 << PCINT1))
	{
		intr = 3;
		ph_b = 9;
		t_idx = 1;
	}
	// else cause firey explosion

	if (digitalRead(intr))
		digitalRead(ph_b) ? tacho[t_idx]++ : tacho[t_idx]--;
	else
		digitalRead(ph_b) ? tacho[t_idx]-- : tacho[t_idx]++;
}

void MotorInit()
{
	// hardcoded kludge
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