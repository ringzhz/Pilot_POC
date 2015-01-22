#include "ArduinoPilot.h"

//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com

char t[64];

int debounceLoops = 30;

// +++ should be time based, eg 1/20 times per second
int checkMqFrequency = 1000;
int checkButtonFrequency = 100;
int CalcPoseFrequency = 1000;
int regulatorFrequency = 200;
long cntr = 0L;

bool esc_enabled = false;

bool useGyro = true;

int debounceCount = 0;
char lastHandledButton;

float Kp, Ki, Kd;

Motor *M1;
Motor *M2;

void setup()
{
	Serial.begin(57600);
	pinMode(LED, OUTPUT);
	
	pinMode(ESC_EN, OUTPUT);

	// +++ calc cycles in a second, set scheduler values

	M1 = new Motor;
	M1->pwmPin = M1_PWM;
	M1->dirPin = M1_DIR;
	M1->intPin = M1_A;
	M1->bPin = M1_B;
	M1->reverse = false;
	M1->tacho = 0L;
	M1->power = 0;
	M1->motorCW = true;
	pinMode(M1_PWM, OUTPUT);
	pinMode(M1_DIR, OUTPUT);
	pinMode(M1_A, INPUT_PULLUP);
	pinMode(M1_B, INPUT_PULLUP);
	attachInterrupt(0, M1_ISR, CHANGE);

	// same for M2
}

int Clip(int a, int low, int high)
{
	return a > high ? high : a < low ? low : a;	
}

int Sign(int v)
{
	return v >= 0 ? 1 : -1;
}

void SetPower(Motor *m, int p)
{
	p = Clip(p, -100, 100);
	if (p != m->power)
	{
		// only set if changed
		m->power = p;
		m->motorCW = p > 0;
		digitalWrite(m->dirPin, m->motorCW);
		analogWrite(m->pwmPin, map(abs(m->power), 0, 100, 0, 255));
	}
}

void M1_ISR()
{
	if (digitalRead(M1->intPin))
		digitalRead(M1->bPin) ? M1->tacho++ : M1->tacho--;
	else
		digitalRead(M1->bPin) ? M1->tacho-- : M1->tacho++;
}

char read_buttons()
{
	int btn = analogRead(0);      // read the value from the sensor 
	return	btn > 1000 ? ' ' :
		btn < 50 ? 'E' :
		btn < 250 ? 'D' :
		btn < 450 ? 'C' :
		btn < 650 ? 'B' :
		btn < 850 ? 'A' :
		' ';  // when all others fail, return this...
}

void CheckButtons()
{
	char btn;
	int p;

	btn = read_buttons();

	//if (btn == lastHandledButton)
	//	return;

	if (--debounceCount > 0)
		return;

	lastHandledButton = btn;

	// kinda weird through 0 (since down always goes towards 0)
	// but since its just stub code, no care
	switch (read_buttons())
	{
	case 'A':
		esc_enabled = !esc_enabled;
		digitalWrite(ESC_EN, esc_enabled);
		debounceCount = debounceLoops;
		break;
	case 'B':
		p = -M1->power;	// quick reverse
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	case 'E':
		// open
		debounceCount = debounceLoops;
		break;
	case 'D':	// up
		p = M1->power + (Sign(M1->power) * 10);
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	case 'C':	// down
		p = M1->power + (-Sign(M1->power) * 10);
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	}
}

void CheckMq()
{

}

void CalcPose()
{
	if (useGyro)
	{
	}
}

void Tick(Motor *m)
{
	// regulator
}

void loop()
{	
	// cheapo scheduler

	// +++check bumper

	// +++check SF ??

	if (cntr % checkMqFrequency == 0)
		CheckMq();

	if (cntr % checkButtonFrequency == 0)
		CheckButtons();

	if (cntr % regulatorFrequency == 0)
	{
		Tick(M1);
		Tick(M2);
	}

	if (cntr % CalcPoseFrequency == 0)
		CalcPose();

	if (cntr % 2000 == 0)
	{
		sprintf(t, "T1 %ld\n", M1->tacho);
		Serial.write(t);
	}

	if (cntr % 5000 == 0)  // blinky
		digitalWrite(LED, !digitalRead(LED));	

	cntr++;

	// +++ before its over, we probably need to handle wrapping better
}
