#include "ArduinoPilot.h"

//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com

// digital pins

#define LED 13

// motor pins
#define M1_PWM 5
#define M2_PWM 6
#define M1_DIR 11
#define M2_DIR 10

#define M1_A 0	// interrupt 0 = pin 2
#define M1_B 8
#define M2_A 1	// interrupt 1 = pin 3
#define M2_B 9

#define ESC_EN 12

// analog pins
#define M1_FB 0
#define M2_FB 1

//#define SDA 4
//#define SCL 5

char t[64];



// +++ should be time based, eg 1/20 times per second
int debounceFrequency = 2;
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

	// +++ calc cycles in a second, set frequencies

	M1 = new Motor;
	M1->pwmPin = M1_PWM;
	M1->dirPin = M1_DIR;
	M1->intPin = M1_A;
	M1->bPin = M1_B;
	M1->reverse = false;

	M2 = new Motor;
	M2->pwmPin = M2_PWM;
	M2->dirPin = M2_DIR;
	M2->intPin = M2_A;
	M2->bPin = M2_B;
	M1->reverse = false;

}

void EncoderInterrupt()
{
	// which pin caused the interrupt?
	// rising or falling?
	// what is current B ?
	// update tacho
	// enable interrupts
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
	bool handled;

	btn = read_buttons();

	if (btn == lastHandledButton)
		return;

	if (--debounceCount > 0)
		return;

	handled = false;
	lastHandledButton = btn;

	switch (read_buttons())
	{
	case 'A':
		esc_enabled = !esc_enabled;
		digitalWrite(ESC_EN, esc_enabled);
		handled = true;
		break;
	case 'B':
		M1->motorCW = false;
		handled = true;
		break;
	case 'E':
		M1->motorCW = true;
		handled = true;
		break;
	case 'D':
		M1->power += 10;
		handled = true;
		break;
	case 'C':
		M1->power -= 10;
		handled = true;
		break;
	}
	if (handled)
		debounceCount = debounceFrequency;
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
	m->power = m->power > 100 ? 100 : m->power < 0 ? 0 : m->power;	// clip
	digitalWrite(m->dirPin, m->motorCW);
	analogWrite(m->pwmPin, m->power);
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
		sprintf(t, "pwr: %d\n", M1->power);
		Serial.write(t);
	}

	if (cntr % 5000 == 0)  // blinky
		digitalWrite(LED, !digitalRead(LED));	

	cntr++;

}
