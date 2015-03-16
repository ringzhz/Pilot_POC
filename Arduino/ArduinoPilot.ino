//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "ArduinoPilot.h"
#include "PilotMotor.h"

//////////////////////////////////////////////////

char read_buttons();
void Log(const char *t);
int Clip(int a, int low, int high);
int Sign(int v);

//////////////////////////////////////////////////

// todo back to motor as object

SoftwareSerial DBG(9, 6);

StaticJsonBuffer<128> jsonBuffer;

int idx = 0;
char serRecvBuf[64];

// pose
double X;
double Y;
double H;

int debounceLoops = 25;

// counter based (ie every X cycles)

int checkMqFrequency = 10;			// byte at a time, so often
int checkButtonFrequency = 120;
int CalcPoseFrequency = 2000;		// +++ aim for 20-30 / sec
int regulatorFrequency = 90;
int hbFrequency = 5000;
int cntr = 0L, counterWrapAt = 30000;

bool esc_enabled = false;
bool useGyro = true;
int debounceCount = 0;
char lastHandledButton = 'X';
float Kp, Ki, Kd;

Geometry Geom;

PilotMotor M1(M1_PWM, M1_DIR, 0, false),
		M2(M2_PWM, M2_DIR, 1, true);

//////////////////////////////////////////////////

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

void Log(const char *t)
{
	// technically, not propper json
	Serial.write("PUBPilot/Log{");
	Serial.write(t);
	Serial.write("}\r\n");

	delay(10);

	DBG.print("PUBPilot/Log{");
	DBG.print(t);
	DBG.print("}\r\n");
}

int Sign(int v)
{
	return v >= 0 ? 1 : -1;
}

//////////////////////////////////////////////////

void setup()
{
	Serial.begin(115200);
	DBG.begin(19200);

	pinMode(LED, OUTPUT);
	pinMode(ESC_EN, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(9, INPUT);

	DBG.print("\r\n\n------\r\n");
	DBG.print("Pilot POC Debug\r\n");

	// +++ calc cycles in a second, set scheduler values

	MotorInit();

	// robot geometry - received data
	// 20 to 1 motor, 3 ticks per motor shaft
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiamter = 175.0;

	digitalWrite(LED, false);

	Serial.write("// please close the AVR serial\r\n");
	Serial.write("// then press 'select' to start\r\n");
	while (true)
	{
		if (read_buttons() == 'A')
			break;
		digitalWrite(LED, true);
		delay(100);
		digitalWrite(LED, false);
		delay(100);
	}

	delay(500);
	Serial.write("SUBPC\n");
	Log("Pilot Running");
}

//////////////////////////////////////////////////

void SetPower(PilotMotor& m, int p)
{
	char t[32];
	p = constrain(p, -100, 100);
	if (p != m.power) // only set if changed
	{
		m.power = p;
		m.motorCW = p >= 0;
		digitalWrite(m.dirPin, m.motorCW);		
		analogWrite(m.pwmPin, map(abs(m.power), 0, 100, 0, 255));
		sprintf(t, "Set Power %d", (int)m.power);
		Log(t);
	}
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

	switch (read_buttons())
	{
	case 'A':
		esc_enabled = !esc_enabled;
		digitalWrite(ESC_EN, esc_enabled);
		Log(esc_enabled ? "Enabled" : "Disabled");
		debounceCount = debounceLoops;
		break;
	case 'B':	// quick reverse
		p = -M1.power;	
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	case 'E':	// not used
		debounceCount = debounceLoops;
		break;
	case 'D':	// up
		p = M1.power + (Sign(M1.power) * 5);
		if (p > 100) p = 100;
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	case 'C':	// down
		p = M1.power + (-Sign(M1.power) * 5);
		if (p < -100) p = -100;
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	}
}

void MqLine(char *line, int l)
{
	DBG.print("MqLine <");
	DBG.print(line);
	DBG.println(">");

	if (strncmp(line, "PC/M1,", 5))
	{
		JsonObject& root = jsonBuffer.parseObject(line + 5);
		SetPower(M1, root["p"]);
	}

	//else if (strncmp(line, "PC/Off,", 6))
	//	SetPower(M1, 0);
}

void CheckMq()
{
	if (Serial.available())
	{
		char c = Serial.read();
		if (c == '\r')
			return;			// ignore
		if (c == '\n')
		{
			MqLine(serRecvBuf, idx);
			memset(serRecvBuf, 0, sizeof(serRecvBuf));
			idx = 0;
		}
		else
			serRecvBuf[idx++] = c;

		if (idx > sizeof(serRecvBuf))
		{
			Log("buffer overrun");
			memset(serRecvBuf, 0, sizeof(serRecvBuf));
			idx = 0;
		}
	}
}

void printDouble(double val, unsigned long precision)
{
	Serial.print(long(val));  //print the int part
	Serial.print(".");
	unsigned long frac;
	if (val >= 0)
		frac = (val - long(val)) * precision;
	else
		frac = (long(val) - val) * precision;

	Serial.print(frac, DEC);
}

void dbgPrintDouble(double val, unsigned long precision)
{
	DBG.print(long(val));  //print the int part
	DBG.print(".");
	unsigned long frac;
	if (val >= 0)
		frac = (val - long(val)) * precision;
	else
		frac = (long(val) - val) * precision;

	DBG.print(frac, DEC);
}

void CalcPose()
{
	if (useGyro)
	{
	}

	// +++ stub
	long currentTacho = M1.GetTacho();
	long delta1 = currentTacho - M1.lastTacho;
	double distance1 = delta1 * Geom.wheelDiamter * PI / Geom.ticksPerRevolution;
	Y += distance1;
	H = 0.0F;

	M1.lastTacho = currentTacho;

	if (delta1 == 0)
		return;	// dont broadacst if no motion.

	//sprintf(t, "PUBPilot/Pose,{\"x\":%f,\"y\":%f,\"h\":%f}\n", X, Y, H);
	Serial.write("PUBPilot/Pose{\"X\":");
	printDouble(X / 100, 100000L);
	Serial.write(",\"Y\":");
	printDouble(Y / 100, 100000L);
	Serial.write(",\"H\":");
	printDouble(H, 100000L);
	Serial.write("}\r\n");

	DBG.write("PUBPilot/Pose{\"X\":");
	dbgPrintDouble(X / 100, 100000L);
	DBG.write(",\"Y\":");
	dbgPrintDouble(Y / 100, 100000L);
	DBG.write(",\"H\":");
	dbgPrintDouble(H, 100000L);
	DBG.write("}\r\n");
}

void Tick(PilotMotor& m)
{
	m.Tick();	// regulator	
}

//////////////////////////////////////////////////

void loop()		// QDHW/SodaPop scheduler
{
	// todo check bumper
	// todo check status flag / amp draw from mc33926

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

	if (cntr % hbFrequency == 0)  // heart beat blinky
		digitalWrite(LED, !digitalRead(LED));

	if (++cntr > counterWrapAt)
		cntr = 0;
}