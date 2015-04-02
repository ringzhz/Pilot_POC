//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

// pc XBee com14 : 9600/8/n/1/n 
// arduino xbee 

#include <digitalWriteFast.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "ArduinoPilot.h"
#include "PilotMotor.h"


#define NO_ESC 1

// digital pins
const int LED = 13;

// motor pins
#if NO_ESC
const int M1_PWM = -1;
const int M2_PWM = -1;
const int M1_DIR = -1;
const int M2_DIR = -1;

SoftwareSerial Gps(5,6);

#else

const int M1_PWM = 5;
const int M2_PWM = 6;
const int M1_DIR = 11;
const int M2_DIR = 17;		// A3

#endif

//#define M1_FB A2
const int M1_FB = 16;
//#define M2_FB A1
const int M2_FB = 15;

char read_buttons();
void Log(const char *t);
int Clip(int a, int low, int high);
int Sign(int v);

//////////////////////////////////////////////////

int  mqIdx = 0;
char mqRecvBuf[128];
char ser2RecvBuf[128];		// for an aux device
char scratch[128];

// pose
double X;
double Y;
double H;		// internally using radians, broadcasts in deggrees

// counter based (ie every X cycles)
int checkGpsFrequency = 10;
int checkMqFrequency = 10;			// we check one byte at a time, so do often
int CalcPoseFrequency = 2000;		// +++ aim for 20-30 / sec
int regulatorFrequency = 100;
int hbFrequency = 5000;
int cntr = 0L, counterWrapAt = 30000;

bool esc_enabled = false;

float Kp1, Ki1, Kd1;

Geometry Geom;

// if pins are set to -1, they will not be used 
// index is an index into the interrupt tacho array for the motor

PilotMotor M1("left", M1_PWM, M1_DIR, M1_FB, 0, false),
M2("right", M2_PWM, M2_DIR, M2_FB, 1, true);

//////////////////////////////////////////////////
// Log delivers a API LogMessage
// Dbg writes to the defined debug stream

void Log(const char *t)
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["Topic"] = "robot1";
	root["T"] = "Log";
	root["Msg"] = t;
	root.printTo(Serial);
	Serial.print('\n');
}

int SignOf(int v)
{
	return v >= 0 ? 1 : -1;
}

//////////////////////////////////////////////////

void setup()
{
	Serial.begin(9600);
	Gps.begin(4800);

	pinMode(LED, OUTPUT);
	pinMode(ESC_EN, OUTPUT);

	Serial.print("//MotorInit ... ");
	MotorInit();
	Serial.print("//complete\n");

	// robot geometry - received data
	// 20 to 1 geared motor, 3 ticks per motor shaft rotation
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 220.0;	
	Geom.EncoderScalar = PI * Geom.wheelDiameter / Geom.ticksPerRevolution;

	digitalWrite(LED, false);

	Serial.print("SUB:Cmd/robot1\n");		// subscribe only to messages targetted to us

	Serial.print("//Pilot Running\n");
	Log("Pilot Running");
}

//////////////////////////////////////////////////

char xx[2] = { '/', '/' };
char gpsBuf[256];
int gpsIdx = 0;

void GpsSentence()
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["Topic"] = "robot1";
	root["T"] = "GPS";
	root["S"] = gpsBuf;
	root.printTo(Serial); Serial.print('\n');
}

void CheckGps()
{

	if (Gps.available() > 0)
	{
		int c = Gps.read();
		gpsBuf[gpsIdx] = c;
		//sprintf(t, "%02x ", c);  Serial.print(t);
		if (c == 0x0a)
		{		
			GpsSentence();
			memset(gpsBuf, 0, gpsIdx);		// only clear as many as we used, save cycles
			gpsIdx = 0;
		}
		else if (++gpsIdx > sizeof(gpsBuf))
		{
			// overflow +++ flush until EOL
			Serial.print("// !! gpsBuf overflow/n");
			gpsIdx = 0;
		}
	}
}

void SetPower(PilotMotor& m, int p)
{
	char t[32];
	if (m.pwmPin != -1)
	{
		p = constrain(p, -100, 100);
		if (p != m.lastPower) // only set if changed
		{
			sprintf(t, "//Set Power %d\n", p); Serial.print(t);
			m.SetSpeed(p);	// +++ stub
		}
	}
}

void cmd_Test(JsonBuffer& j)
{
	Log("robot1::Test");	
	Serial.print("//robot1::Test\n");
}

void ResetPose()
{
	Log("robot1::ResetPose");
	Serial.print("//ResetPose\n");
	M1.Reset();
	M2.Reset();
	X = Y = H = 0.0;
}

void MqLine(char *line, int l)
{
	char t[64];
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.parseObject(line);
	
	sprintf(t, "//rcv <- line(%s), root['T'](%s) \n", line, root["T"]);  Serial.print(t);

	if (strcmp(root["T"], "Cmd") == 0)
	{
		if (strcmp(root["Cmd"], "Reset") == 0)
			ResetPose();
		else if (strcmp(root["Cmd"], "Test1") == 0)
			cmd_Test(jsonBuffer);
	}
	else
		Serial.print("//rcv <- missing or unrecognized type(T)\n");
}

void CheckMq()
{
	if (Serial.available())
	{
		char c = Serial.read();
		if (c == '\r')		// ignore
			return;			
		if (c == '\n')		// end of line, process
		{
			MqLine(mqRecvBuf, mqIdx);
			memset(mqRecvBuf, 0, sizeof(mqRecvBuf));
			mqIdx = 0;
		}
		else
			mqRecvBuf[mqIdx++] = c;

		if (mqIdx > sizeof(mqRecvBuf))
		{
			Serial.write("// !! buffer overrun");
			memset(mqRecvBuf, 0, sizeof(mqRecvBuf));
			mqIdx = 0;
		}
	}
}

void PublishPose()
{
	StaticJsonBuffer<128> jsonBuffer;
#if 1
	JsonObject& root = jsonBuffer.createObject();
	root["Topic"] = "robot1";
	root["T"] = "Tach";
	root["M1"].set(M1.GetTacho(), 0);  // 0 is the number of decimals to print
	root["M2"].set(M2.GetTacho(), 0);
	root.printTo(Serial); Serial.print('\n');	

#endif
	JsonObject& root2 = jsonBuffer.createObject();
	root2["Topic"] = "robot1";
	root2["T"] = "Pose";
	root2["X"].set(X, 6);
	root2["Y"].set(Y, 6);
	root2["H"].set(RAD_TO_DEG * H, 4);
	root2.printTo(Serial); Serial.print('\n');
}

bool CalcPose()
{
	// +++ add gyro integration/kalman

	long tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	long delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	if (abs(delta1) + abs(delta2) < 1)
		return false;	// no significant movement

	double delta = (delta2 + delta1) * Geom.EncoderScalar / 2.0;
	// +++ not sure why I needed to add * 2.0 here, but it worked - but keep an eye on it
	double headingDelta = (delta2 - delta1) * 2.0 / Geom.wheelBase;

	X += delta * sin(H + headingDelta / 2.0);
	Y += delta * cos(H + headingDelta / 2.0);
	H += headingDelta;
	H = fmod(H, TWO_PI);

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;

	return true;
}

//////////////////////////////////////////////////

void loop()
{
	// todo check bumper / ultrasonic / gyro data rdy
	// todo check status flag / amp draw from mc33926

	if (cntr % checkMqFrequency == 0)
		CheckMq();

	if (cntr % checkGpsFrequency == 0)
		CheckGps();

	if (cntr % regulatorFrequency == 0)
	{
		M1.Tick();
		M2.Tick();
	}

	if (cntr % CalcPoseFrequency == 0)
		if (CalcPose())
			PublishPose();

	if (cntr % hbFrequency == 0)  // heart beat blinky
		digitalWrite(LED, !digitalRead(LED));

	if (++cntr > counterWrapAt)
		cntr = 0;
}

//////////////////////////////////////////////////

#if 0
// old breadboard keyboard stuff - may need again someday
int debounceLoops = 25;
int checkButtonFrequency = 120;
int debounceCount = 0;
char lastHandledButton = 'X';


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

#if 0
Serial.write("// please close the AVR serial\r\n");

Serial.write("// then press 'select' to start\r\n");
while (true)
{
	if (read_buttons() == 'A')
		break;
	toggle(LED);
	delay(100);
}
delay(500);
#endif
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

if (cntr % checkButtonFrequency == 0)
CheckButtons();

#endif