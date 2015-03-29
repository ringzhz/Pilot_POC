//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

// pc XBee com14 : 9600/8/n/1/n 
// arduino xbee 

#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "ArduinoPilot.h"
#include "PilotMotor.h"

//////////////////////////////////////////////////
// +++ NO SPEED CONTROLLER / using XBEE version
//////////////////////////////////////////////////

const int XBeeRx = 10;
const int XBeeTx = 11;

SoftwareSerial XB(XBeeRx, XBeeTx);

char read_buttons();
void Log(const char *t);
int Clip(int a, int low, int high);
int Sign(int v);

//////////////////////////////////////////////////

char scratch[128];

int idx = 0;
char serRecvBuf[256];

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
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["Topic"] = "robot1//Log";
	root["Msg"] = t;	
	root.printTo(XB);
	XB.print('\n');
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
	Dbg("..MotorInit finished");

	// robot geometry - received data
	// 20 to 1 motor, 3 ticks per motor shaft
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 220.0;	
	Geom.EncoderScalar = PI * Geom.wheelDiameter / Geom.ticksPerRevolution;

	digitalWrite(LED, false);

<<<<<<< HEAD
	/*Serial.write("// please close the AVR serial\r\n");
=======
#if 0
	Serial.write("// please close the AVR serial\r\n");
>>>>>>> no-esc
	Serial.write("// then press 'select' to start\r\n");
	while (true)
	{
		if (read_buttons() == 'A')
			break;
		toggle(LED);
		delay(100);
<<<<<<< HEAD
	}
*/
=======
	}	
>>>>>>> no-esc
	delay(500);
#endif

	XB.write("SUB:PC/robot1/#\n");		// only messages targetted to us

	Dbg("Pilot Running");
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
			Dbg(t);
		}
	}
}

char btn = 'Z';
void CheckButtons()
{
	int p;

	if (btn == 'Z')
		btn = 'A';
	else if (btn == 'A')
	{
		SetPower(M1, 50);
		btn = 'X';
	}
		

	//btn = read_buttons();

	//if (--debounceCount > 0)
	//	return;

	lastHandledButton = btn;

	switch (btn)
	{
	case 'A':
		esc_enabled = !esc_enabled;
		digitalWrite(ESC_EN, esc_enabled);
		Serial.write(esc_enabled ? "Enabled" : "Disabled");
		debounceCount = debounceLoops;
		break;
	case 'B':	// instant reverse
		p = -M1.power;	
		SetPower(M2, p);
		debounceCount = debounceLoops;
		break;
	case 'E':	// not used
		debounceCount = debounceLoops;
		break;
	case 'D':	// up
		p = M1.power + (SignOf(M1.power) * 5);
		if (p > 100) p = 100;
		SetPower(M2, p);
		debounceCount = debounceLoops;
		break;
	case 'C':	// down
		p = M1.power + (-SignOf(M1.power) * 5);
		if (p < -100) p = -100;
		SetPower(M2, p);
		debounceCount = debounceLoops;
		break;
	}
}

void MqLine(char *line, int l)
{
	// +++ msg format has changed - we now only get messages targeting us
<<<<<<< HEAD
	JsonObject& root = jsonBuffer.parseObject(line + 5);
	if (strncmp(root["t"], "M2", 2))
		SetPower(M2, root["pwr"]);
=======
	StaticJsonBuffer<256> jsonBuffer;
	JsonObject& root = jsonBuffer.parseObject(line);
	if (strncmp(root["Topic"], "M1", 2))
		SetPower(M1, root["Power"]);
>>>>>>> no-esc
}

void CheckMq()
{
	if (XB.available())
	{
		char c = XB.read();
		if (c == '\r')		// ignore
			return;			
		if (c == '\n')		// end of line, process
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

void PublishPose()
{
	StaticJsonBuffer<256> jsonBuffer;
#if 1
	JsonObject& root = jsonBuffer.createObject();
	root["Topic"] = "robot1/Tach";
	root["M1"].set(M1.GetTacho(), 0);  // 0 is the number of decimals to print
	root["M2"].set(M2.GetTacho(), 0);
	root.printTo(XB);
	XB.print('\n');	

#endif
	JsonObject& root2 = jsonBuffer.createObject();
	root2["Topic"] = "robot1/Pose";
	root2["X"].set(X, 6);
	root2["Y"].set(Y, 6);
	root2["H"].set(RAD_TO_DEG * H, 4);
	root2.printTo(XB);
	XB.print('\n');
}

bool CalcPose()
{
	// +++ add gyro integration/kalman

	long delta1 = M1.GetTacho() - M1.lastTacho,
		delta2 = M2.GetTacho() - M2.lastTacho;

	if (abs(delta1) + abs(delta2) < 1)
		return false;	// no significant movement

	double delta = (delta2 + delta1) * Geom.EncoderScalar / 2.0;
	double headingDelta = (delta2 - delta1) * 2.0 / Geom.wheelBase;

	X += delta * sin(H + headingDelta / 2.0);
	Y += delta * cos(H + headingDelta / 2.0);
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