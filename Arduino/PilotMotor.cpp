#import <Arduino.h>
#include "PilotMotor.h"

volatile long tacho[2];		// interrupt 0 and interrupt 1 tachos

ISR(MotorISR1)
{
	// +++ fastRead
	//char c = PIND;
	//if (c & (1 << PD2))
	//	(c & (1 << PD))
	//int b = bitRead()
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
	// hardcoded
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);

	attachInterrupt(PCINT0, MotorISR1, CHANGE);
	attachInterrupt(PCINT1, MotorISR2, CHANGE);
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

	if (pwm > -1)
	{
		pinMode(pwm, OUTPUT);
		pinMode(dir, OUTPUT);
	}
	tacho[interruptIndex] = 0L;
}

long PilotMotor::GetTacho()
{
	return reverse ? -tacho[interruptIndex] : tacho[interruptIndex];
}

void PilotMotor::Tick()
{

}