//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

// pc XBee com14 / 115.2/8/n/1/n source addr 4, dest 3
// arduino xbee 

#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "ArduinoPilot.h"
#include "PilotMotor.h"

//////////////////////////////////////////////////

// +++ now-esc version
const int XBeeRx = 10;
const int XBeeTx = 11;

SoftwareSerial XB(XBeeRx, XBeeTx);

char read_buttons();
void Log(const char *t);
int Clip(int a, int low, int high);
int Sign(int v);

//////////////////////////////////////////////////

char scratch[128];
StaticJsonBuffer<128> jsonBuffer;

int idx = 0;
char serRecvBuf[64];

// pose
double X;
double Y;
double H;		// internally using radians, broadcast in deggrees

int debounceLoops = 25;

// counter based (ie every X cycles)

int checkMqFrequency = 10;			// we check one byte at a time, so do often
int checkButtonFrequency = 120;
int CalcPoseFrequency = 2000;		// +++ aim for 20-30 / sec
int regulatorFrequency = 90;
int hbFrequency = 5000;
int cntr = 0L, counterWrapAt = 30000;

bool esc_enabled = false;
int debounceCount = 0;
char lastHandledButton = 'X';
float Kp, Ki, Kd;

Geometry Geom;

PilotMotor M1(-1, -1, 0, false),
		M2(-1, -1, 1, true);

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
	XB.write("{t=\"robot1/Log,txt=\"");
	XB.write(t);
	XB.write("\"}\r\n");
	delay(10);
}

void Dbg(const char *t)
{
	Serial.write(t);
}

int SignOf(int v)
{
	return v >= 0 ? 1 : -1;
}

//////////////////////////////////////////////////

void setup()
{
	Serial.begin(57600);
	XB.begin(9600);

	pinMode(LED, OUTPUT);
	pinMode(ESC_EN, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(9, INPUT);

	// +++ calc cycles in a second, set scheduler values
	Dbg("MotorInit");
	MotorInit();

	// robot geometry - received data
	// 20 to 1 motor, 3 ticks per motor shaft
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 200.0;	
	Geom.EncoderScalar = PI * Geom.wheelDiameter / Geom.ticksPerRevolution;

	digitalWrite(LED, false);

	//Serial.write("// please close the AVR serial\r\n");
	//Serial.write("// then press 'select' to start\r\n");
	//while (true)
	//{
	//	if (read_buttons() == 'A')
	//		break;
	//	toggle(LED);
	//	delay(100);
	//}

	//delay(500);

	XB.write("SUB{t=\"host/robot1/#\"}\r\n");
	Log("Pilot Running");
}

//////////////////////////////////////////////////

void SetPower(PilotMotor& m, int p)
{
	char t[32];
	if (m.pwmPin > -1)
	{
		p = constrain(p, -100, 100);
		if (p != m.power) // only set if changed
		{
			m.power = p;
			m.motorCW = p >= 0;
			digitalWrite(m.dirPin, m.motorCW);
			analogWrite(m.pwmPin, map(abs(m.power), 0, 100, 0, 255));
			sprintf(t, "Set Power %d", (int)m.power);
			Serial.write(t);
		}
	}
}

void CheckButtons()
{
	char btn;
	int p;

	btn = read_buttons();

	if (--debounceCount > 0)
		return;

	lastHandledButton = btn;

	switch (read_buttons())
	{
	case 'A':
		esc_enabled = !esc_enabled;
		digitalWrite(ESC_EN, esc_enabled);
		Serial.write(esc_enabled ? "Enabled" : "Disabled");
		debounceCount = debounceLoops;
		break;
	case 'B':	// instant reverse
		p = -M1.power;	
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	case 'E':	// not used
		debounceCount = debounceLoops;
		break;
	case 'D':	// up
		p = M1.power + (SignOf(M1.power) * 5);
		if (p > 100) p = 100;
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	case 'C':	// down
		p = M1.power + (-SignOf(M1.power) * 5);
		if (p < -100) p = -100;
		SetPower(M1, p);
		debounceCount = debounceLoops;
		break;
	}
}

void MqLine(char *line, int l)
{
	// +++ msg format has changed - we now only get messages targeting us
	JsonObject& root = jsonBuffer.parseObject(line + 5);
	if (strncmp(root["t"], "M1", 2))
		SetPower(M1, root["pwr"]);
}

void CheckMq()
{
	if (XB.available())
	{
		char c = XB.read();
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
			Serial.write("buffer overrun");
			memset(serRecvBuf, 0, sizeof(serRecvBuf));
			idx = 0;
		}
	}
}

// +++ string appender kind of thing would be better
void printDouble(double val, unsigned long precision)
{
	XB.print(long(val));  //print the int part
	XB.print(".");
	unsigned long frac;
	if (val >= 0)
		frac = (val - long(val)) * precision;
	else
		frac = (long(val) - val) * precision;

	XB.print(frac, DEC);
}

void PublishPose()
{
#if 1
	sprintf(scratch, "{t=\"robot1/Tach\", M1: %ld, M2: %ld}\r\n", M1.GetTacho(), M2.GetTacho());
	XB.write(scratch);
#endif
	XB.write("{t=\"robot1/Pose\",X:");
	printDouble(X / 100, 100000L);
	XB.write(",Y:");
	printDouble(Y / 100, 100000L);
	XB.write(",H:");
	printDouble((H * RAD_TO_DEG), 100000L);
	XB.write("}\r\n");
}

bool CalcPose()
{
	// +++ add gyro integration/kalman

	long delta1 = M1.GetTacho() - M1.lastTacho,
		delta2 = M2.GetTacho() - M2.lastTacho;

	if (abs(delta1) + abs(delta2) < 1)
		return false;	// no significant movement

	double delta = (delta2 + delta1) * Geom.EncoderScalar / 2.0;
	double headingDelta = (delta2 - delta1) / Geom.wheelBase;

	X += delta * cos(H + headingDelta / 2.0);
	Y += delta * sin(H + headingDelta / 2.0);
	H += headingDelta;
	H = fmod(H, PI * 2.0);

	M1.lastTacho = M1.GetTacho();
	M2.lastTacho = M2.GetTacho();

	return true;
}


//////////////////////////////////////////////////

void loop()
{
	// todo check bumper / ultrasonic / gyro data rdy
	// todo check status flag / amp draw from mc33926

	if (cntr % checkMqFrequency == 0)
		CheckMq();

	if (cntr % checkButtonFrequency == 0)
		CheckButtons();

	if (cntr % regulatorFrequency == 0)
	{
		M1.Tick();
		M2.Tick();
	}

	if (cntr % CalcPoseFrequency == 0)
	{
		if (CalcPose())
			PublishPose();
	}

	if (cntr % hbFrequency == 0)  // heart beat blinky
		digitalWrite(LED, !digitalRead(LED));

	if (++cntr > counterWrapAt)
		cntr = 0;
}