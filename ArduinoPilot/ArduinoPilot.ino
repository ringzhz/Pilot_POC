//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include <digitalWriteFast.h>
#include <wire.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#include "ArduinoPilot.h"
#include "PilotMotor.h"
#include "Commands.h"
#include "Broadcasts.h"

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

bool mpuEnabled = true;
bool useGyro = false;
bool escEnabled = false;
bool gpsEnabled = true;
bool heartbeatEnabled = false;

//////////////////////////////////////////////////

// counter based (ie every X cycles)
int checkGpsFrequency = 7;
int checkMpuFrequency = 49;  // +++ should be not used, pin driven
int checkMqFrequency = 8;			// we check one byte at a time, so do often
int CalcPoseFrequency = 1920;		// +++ aim for 20-30 / sec
int regulatorFrequency = 99;
int hbFrequency = 5000;
long cntr = 0L, counterWrapAt = 214748363L;

// mpu ////////////////////////////////////////////////

const int mpuInterruptPin = 7;
bool mpuInterrupt = false;

const int MPU = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t AcXOffset, AcYOffset, AcZOffset, TmpOffset, GyXOffset, GyYOffset, GyZOffset = 85;

// gps ////////////////////////////////////////////////

int gpsIdx = 0;
char gpsBuf[96]; // max is 82
SoftwareSerial Gps(5, 6);

// motor ////////////////////////////////////////////////

double X = 0.0;		// internally mm, broadcast in meters
double Y = 0.0;
double H = 0.0;		// internally using radians, broadcasts in deggrees

Geometry Geom;

// +++ change so uses escEnabled instead of -1
// if pins are set to -1, they will not be used, index is the tacho interrupt array
PilotMotor M1("left", M1_PWM, M1_DIR, M1_FB, 0, false),
M2("right", M2_PWM, M2_DIR, M2_FB, 1, true);

float Kp1, Ki1, Kd1;

// messageQ ////////////////////////////////////////////////

int  mqIdx = 0;
char mqRecvBuf[128];

/////////////////////////////////////////////////////


CmdFunction cmdTable[] {
	{ "Rest",	cmd_Reset },
	{ "Esc",	cmd_Esc },
	{ "Geom",	cmd_Geom },
	{ "Move",	cmd_Move },
	{ "GPS",	cmd_GPS },
	{ "Test1",	cmd_Test1 },
	{ "Test2",	cmd_Test2 },
};

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

	pinMode(LED, OUTPUT);
	pinMode(ESC_EN, OUTPUT);
	digitalWrite(LED, false);
	digitalWrite(ESC_EN, false);

	MotorInit();	// interrupt handler(s) always on

	if (mpuEnabled)
	{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
#endif
		//pinMode(mpuInterruptPin, INPUT_PULLUP);
		Wire.beginTransmission(MPU);
		Wire.write(0x6B);  // PWR_MGMT_1 register
		Wire.write(0);     // set to zero (wakes up the MPU-6050)
		Wire.endTransmission(true);
	}

	if (escEnabled)
	{
		pinMode(M1_PWM, OUTPUT);
		digitalWrite(M1_PWM, 0);
		pinMode(M2_PWM, OUTPUT);
		digitalWrite(M2_PWM, 0);
		pinMode(M1_DIR, OUTPUT);
		digitalWrite(M1_DIR, 0);
		pinMode(M2_DIR, OUTPUT);
		digitalWrite(M2_DIR, 0);
		M1.Reset();
		M2.Reset();
	}

	if (gpsEnabled)
	{
		Gps.begin(4800);
	}

	// robot geometry - received data
	// 20 to 1 geared motor, 3 ticks per motor shaft rotation
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 220.0;	
	Geom.EncoderScalar = PI * Geom.wheelDiameter / Geom.ticksPerRevolution;

	Serial.print("SUB:Cmd/robot1\n");		// subscribe only to messages targetted to us

	const char build[] = "Pilot V2R1.04 (gyro1)";

	Serial.print("// ");
	Serial.print(build);
	Serial.print("\n");
	Log(build);
}

//////////////////////////////////////////////////

/*
0 ± 250 ° / s 131 LSB / ° / s		default
1 ± 500 ° / s 65.5 LSB / ° / s
2 ± 1000 ° / s 32.8 LSB / ° / s
3 ± 2000 ° / s 16.4 LSB / ° / s
*/

const float gyroSensitivity = 131.0F;
unsigned long millisLastMpu = 0;
unsigned long microsLastMpu = 0;

void checkMpu()
{
	// +++ we need to go through a calibrate phase, which starts at least 10 seconds AFTER power on
	// the subtract offsets, and a configurable drift compensation?
	// but we are basically ready for Kalman
	Wire.beginTransmission(MPU);
	Wire.write(0x47);  // starting with register
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 2, true);  // request a total of 
	//GyX = Wire.read() << 8 | Wire.read();  // 0x43/44 X
	//GyY = Wire.read() << 8 | Wire.read();  // 0x45/46 Y
	GyZ = Wire.read() << 8 | Wire.read();  // 0x47/48 Z
	//Serial.print(" GyX = "); Serial.print(GyX / gyroSensitivity);
	//Serial.print(" | GyY = "); Serial.print(GyY / gyroSensitivity);
	Serial.print(" | GyZ = "); Serial.println((GyZ - GyZOffset) / gyroSensitivity);

	// +++ integrate
	
	unsigned t = micros();		// handle wrap (approx 70 minutes)!!
	if (t < microsLastMpu)
		millisLastMpu++;
	microsLastMpu = t;
	mpuInterrupt = false;
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
		if (strcmp(cmdTable[i].cmd, (const char *)j["Cmd"]) == 0)
			bool rc = (*cmdTable[i].f)(j);
}

void MqLine(char *line, int l)
{
	char t[64];
	const char *T;
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& j = jsonBuffer.parseObject(line);

	if (strcmp((const char *)j["T"], "Cmd") == 0)
		ProcessCommand(j);
	else
	{
		sprintf(t, "//rcv <- missing or unrecognized T \"%s\"\n", j["T"]); Serial.print(t);
	}
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
			memset(mqRecvBuf, 0, mqIdx);
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
	// todo check bumper / ultrasonic 
	// todo check status flag / amp draw from mc33926

	if (cntr % checkMqFrequency == 0)
		CheckMq();

	if (gpsEnabled && (cntr % checkGpsFrequency == 0) )
		CheckGps();

//	if (digitalReadFast(mpuInterruptPin))
	if (cntr % checkMpuFrequency == 0)
		mpuInterrupt = true;

	if (mpuEnabled && mpuInterrupt)
		checkMpu();

	if (escEnabled && (cntr % regulatorFrequency == 0))		// PID regulator
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
		if (heartbeatEnabled && digitalRead(LED))
			Serial.print("{\"Topic\":\"robot1\", \"T\":\"HeartBeat\"}\n");
	}

	if (++cntr > counterWrapAt)
		cntr = 0;
}

