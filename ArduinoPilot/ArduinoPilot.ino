//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <digitalWriteFast.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "helper_3dmath.h"

#include "PilotMotor.h"
#include "Commands.h"
#include "broadcasts.h"
#include "ArduinoPilot.h"
#include "pose.h"

const int LED = 13;

const int ESC_EN = 12;

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

/// Globals ///////////////////////////////////////////////

const char *Topic = "Topic";

bool AhrsEnabled = true;
bool escEnabled = false;
bool heartbeatEnabled = true;

Geometry Geom;

//////////////////////////////////////////////////

// counter based (ie every X cycles)
int checkGpsFrequency = 7;
int checkMpuFrequency = 49;  // +++ should be not used, pin driven
int checkMqFrequency = 8;			// we check one byte at a time, so do often
int CalcPoseFrequency = 1000;		// +++ aim for 20-30 / sec
int regulatorFrequency = 99;
int hbFrequency = 500;
long cntr = 0L, counterWrapAt = 214748363L;

// mpu ////////////////////////////////////////////////

MPU6050 mpu;

const int mpuInterruptPin = 7;
bool mpuInterrupt = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// gps ////////////////////////////////////////////////

// +++ change so uses escEnabled instead of -1
// if pins are set to -1, they will not be used, index is the tacho interrupt array
PilotMotor M1("left", M1_PWM, M1_DIR, M1_FB, 0, false), M2("right", M2_PWM, M2_DIR, M2_FB, 1, true);

float Kp1, Ki1, Kd1;

// messageQ ////////////////////////////////////////////////

int  mqIdx = 0;
char mqRecvBuf[64];

/////////////////////////////////////////////////////

CmdFunction cmdTable[] {
	{ "Rest", cmd_Reset },
	{ "Esc", cmd_Esc },
	{ "Geom", cmd_Geom },
	{ "Move", cmd_Move },
	{ "GPS", cmd_GPS },
	{ "Test1", cmd_Test1 },
	{ "Test2", cmd_Test2 },
};

//////////////////////////////////////////////////

void Log(const char *t)
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Log";
	root["Msg"] = t;
	root.printTo(Serial);
	Serial.print('\n');
}

//////////////////////////////////////////////////

void setup()
{
	char t[64];

	Serial.begin(9600);

	pinMode(LED, OUTPUT);
	pinMode(ESC_EN, OUTPUT);
	digitalWrite(LED, false);
	digitalWrite(ESC_EN, false);

	MotorInit();	// interrupt handler(s) always on

	if (AhrsEnabled)
	{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
#endif

		// initialize device
		Serial.println(F("// Initializing I2C devices..."));
		mpu.initialize();

		// verify connection
		Serial.println(F("// Testing device connections..."));
		Serial.println(mpu.testConnection() ? F("// MPU6050 connection successful") : F("// MPU6050 connection failed"));

		// wait for ready
		Serial.println(F("// \nSend any character to begin DMP programming and demo: "));
		while (Serial.available() && Serial.read()); // empty buffer
		while (!Serial.available());                 // wait for data
		while (Serial.available() && Serial.read()); // empty buffer again

		// load and configure the DMP
		Serial.println(F("// Initializing DMP..."));
		devStatus = mpu.dmpInitialize();

		// supply your own gyro offsets here, scaled for min sensitivity
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

		// make sure it worked (returns 0 if so)
		if (devStatus == 0) {
			// turn on the DMP, now that it's ready
			Serial.println(F("Enabling DMP..."));
			mpu.setDMPEnabled(true);

			// enable Arduino interrupt detection
			Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
			// +++			attachInterrupt(0, dmpDataReady, RISING);
			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			Serial.println(F("DMP ready! Waiting for first interrupt..."));
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
		}
		else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
		}
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

	// robot geometry - received data
	// 20 to 1 geared motor, 3 ticks per motor shaft rotation
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 220.0;
	Geom.EncoderScaler = PI * Geom.wheelDiameter / Geom.ticksPerRevolution;

	Serial.print(F("SUB:Cmd/robot1\n"));		// subscribe only to messages targetted to us

	delay(200);

	Serial.println(F("// Pilot V2R1.04 (gyro1)"));
}

//////////////////////////////////////////////////

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

//////////////////////////////////////////////////

void loop()
{
	// todo check bumper / ultrasonic
	// todo check status flag / amp draw from mc33926

	if (cntr % checkMqFrequency == 0)
		CheckMq();

	//	if (digitalReadFast(mpuInterruptPin))
	//if (cntr % checkMpuFrequency == 0)
	//	mpuInterrupt = true;

	//if (mpuEnabled && mpuInterrupt)
	//	checkMpu();

	if (escEnabled && (cntr % regulatorFrequency == 0))		// PID regulator
	{
		M1.Tick();
		M2.Tick();
	}

	if (cntr % CalcPoseFrequency == 0)
	{
		bool r;
		r = CalcPose();
		if (r)
			PublishPose();
	}

	if (cntr % hbFrequency == 0)  // heart beat blinky
	{
		digitalWrite(LED, !digitalRead(LED));
		if (heartbeatEnabled && digitalRead(LED))
			PublishHeartbeat();
	}

	if (++cntr > counterWrapAt)
		cntr = 0;
}