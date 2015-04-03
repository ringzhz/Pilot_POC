//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved


#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "ArduinoPilot.h"
#include "PilotMotor.h"
#include "Commands.h"
#include "broadcasts.h"

#define NO_ESC 0

// digital pins
const int LED = 13;

// motor pins
const int M1_PWM = 5;
const int M2_PWM = 6;
const int M1_DIR = 11;
const int M2_DIR = 17;		// A3

//#define M1_FB A2
const int M1_FB = 16;
//#define M2_FB A1
const int M2_FB = 15;

void Log(const char *t);
#define Sign(A) (A >= 0 ? 1 : -1)

//////////////////////////////////////////////////

int  mqIdx = 0;
char mqRecvBuf[128];

int gpsIdx = 0;
char gpsBuf[128];

char ser2RecvBuf[128];		// for an aux device

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
long cntr = 0L, counterWrapAt = 214748363L;

bool publishHB_enabled = false;

float Kp1, Ki1, Kd1;

Geometry Geom;

CmdFunction cmdTable[] {
	{ "Rest",	cmd_Reset },
	{ "ESC",	cmd_Esc },
	{ "Geom",	cmd_Geom },
	{ "Move",	cmd_Move },
	{ "GPS",	cmd_GPS },
	{ "Test1",	cmd_Test1 },
	{ "Test2",	cmd_Test2 },
};

// if pins are set to -1, they will not be used
// index is an index into the interrupt tacho array for the motor

bool esc_enabled = false;

PilotMotor M1("left", M1_PWM, M1_DIR, M1_FB, 0, false),
	M2("right", M2_PWM, M2_DIR, M2_FB, 1, true);

bool gps_enabled = false;

SoftwareSerial Gps(5, 6);

//////////////////////////////////////////////////

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

//////////////////////////////////////////////////

void setup()
{
	Serial.begin(9600);
	Gps.begin(4800);

	pinMode(LED, OUTPUT);
	pinMode(ESC_EN, OUTPUT);
	digitalWrite(LED, false);
	digitalWrite(ESC_EN, false);

	MotorInit();	// just initializes interrupt handler(s)

	pinMode(M1_PWM, OUTPUT);
	digitalWrite(M1_PWM, 0);
	pinMode(M2_PWM, OUTPUT);
	digitalWrite(M2_PWM, 0);
	pinMode(M1_DIR, OUTPUT);
	digitalWrite(M1_DIR, 0);
	pinMode(M2_DIR, OUTPUT);
	digitalWrite(M2_DIR, 0);

	// robot geometry - received data
	// 20 to 1 geared motor, 3 ticks per motor shaft rotation
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 220.0;	
	Geom.EncoderScalar = PI * Geom.wheelDiameter / Geom.ticksPerRevolution;

	Serial.print("SUB:Cmd/robot1\n");		// subscribe only to messages targetted to us

	Serial.print("//Pilot V1R3.00 Running\n");
	Log("Pilot V1R3.00 Running\n");
}

//////////////////////////////////////////////////

void CheckGps()
{
	if (Gps.available() > 0)
	{
		int c = Gps.read();
		gpsBuf[gpsIdx] = c;
		//sprintf(t, "%02x ", c);  Serial.print(t);
		if (c == 0x0a)
		{		
			PublishGps();
			memset(gpsBuf, 0, gpsIdx);		// only clear as many as we used, save cycles
			gpsIdx = 0;
		}
		else if (++gpsIdx > sizeof(gpsBuf))
		{
			// overflow +++ flush until EOL
			Serial.print("// !! gpsBuf overflow !!/n");
			gpsIdx = 0;
		}
	}
}

void ProcessCommand(JsonObject& j)
{
	for (int i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
		if (strcmp(cmdTable[i].cmd, (const char *)j["T"]) == 0)
			bool rc = (*cmdTable[i].f)(j);
}

void MqLine(char *line, int l)
{
	char t[64];
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.parseObject(line);
	
	sprintf(t, "//rcv <- line(%s), root['T'](%s) \n", line, root["T"]);  Serial.print(t);

	if (strcmp(root["T"], "Cmd") == 0)
		ProcessCommand(root);
	else
		Serial.print("//rcv <- missing or unrecognized T (type)\n");
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

	if (gps_enabled && (cntr % checkGpsFrequency == 0))
		CheckGps();

	if (esc_enabled && (cntr % regulatorFrequency == 0))
	{
		M1.Tick();
		M2.Tick();
	}

	if (cntr % CalcPoseFrequency == 0)
		if (CalcPose())
			PublishPose();

	if (cntr % hbFrequency == 0)  // heart beat blinky
	{
		digitalWrite(LED, !digitalRead(LED));
		if (publishHB_enabled && digitalRead(LED))
			Serial.print("{\"Topic\":\"robot1\", \"T\":\"HeartBeat\"}\n");
	}

	if (++cntr > counterWrapAt)
		cntr = 0;
}

