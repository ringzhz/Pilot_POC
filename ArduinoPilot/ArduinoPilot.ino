//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright (c) 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <Wire.h>
#include <ArduinoJson.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "digitalWriteFast.h"

#if 1 
#define DBGLOG(x) Serial.print(x)
#else
#define DBGLOG(x)
#endif

// pins are defines to allow feastRead/Writes
// interrupt/phase pins are hardcoded in motor.cpp
#define BUMPER 4
#define LED 13
#define ESC_ENA 12
#define MPU_INT 7

// motor pins
#define M1_PWM 5
#define M2_PWM 6
#define M1_DIR 11
#define M2_DIR 17	// A3
#define M1_FB  16	// A2
#define M2_FB  15	// A1

const char *Topic = "Topic";

////////////////////////////////////////////////////////////

typedef struct {
	int ticksPerRevolution;
	float wheelDiameter;
	float wheelBase;
	float EncoderScaler;	// calculated
} Geometry;

float X = 0.0;		// internally mm, published in meters
float Y = 0.0;
float H = 0.0;		// internally using radians, published in degrees

uint64_t LastPoseTime = 0L;
float previousYaw = 0.0;
uint8_t lastBumperRead = 0xff;

bool AhrsEnabled = true;
bool escEnabled = true;
bool heartbeatEventEnabled = false;
bool BumperEventEnabled = true;
bool DestinationEventEnabled = true;
bool pingEventEnabled = false;
bool PoseEventEnabled = true;

// counter based (ie every X cycles)
uint16_t CalcPoseFrequency = 100;		// +++ aim for 20-30 / sec
uint16_t regulatorFrequency = 100;
uint16_t motorRegulatorFrequency = 50;
uint16_t heartbeatEventFrequency = 5000;
uint64_t cntr = 0L;

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

#include "motor.h"
#include "commands.h"
#include "publish.h"

Geometry Geom;
MPU6050 mpu;

PilotMotor	M1("M1", M1_PWM, M1_DIR, M1_FB, 0, false),
			M2("M2", M2_PWM, M2_DIR, M2_FB, 1, true);

int  mqIdx = 0;
char mqRecvBuf[128];
bool dmp_rdy = false;

// FYI
//#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
//#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
//#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

////////////////////////////////////////////////////////////

void Log(String t)
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Log";
	root["Msg"] = t.c_str();
	root.printTo(Serial);
	Serial.print('\n');
}

// escEnabled serves 2 purposes. if it is false at startup, the pins are not initialized
// after startup it is used to actually enable/disable the speed controlers

void setup()
{
	Serial.begin(115200, SERIAL_8N1);
	Serial.print(F("// Pilot V2R1.05 (gyro1.1)\n"));

	pinMode(LED, OUTPUT);
	pinMode(ESC_ENA, OUTPUT);

	digitalWrite(LED, false);
	digitalWrite(ESC_ENA, false);

	MotorInit();	// interrupt handler(s), pinmode(s)

	if (AhrsEnabled)
	{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
#endif
		pinMode(MPU_INT, INPUT_PULLUP);

		Serial.print(F("// Initializing I2C devices...\n"));
		mpu.initialize();

		Serial.print(F("// Testing device connections...\n"));
		Serial.print(mpu.testConnection() ? F("// MPU6050 connection successful\n") :
			F("// MPU6050 connection failed\n"));

		// wait for ready
		delay(400);

		Serial.println(F("// Initializing DMP...\n"));
		devStatus = mpu.dmpInitialize();

		// +++ supply your own gyro offsets here, scaled for min sensitivity
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

		if (devStatus == 0) {
			mpu.setDMPEnabled(true);
			mpuIntStatus = mpu.getIntStatus();
			Serial.print(F("// DMP ready\n"));
			packetSize = mpu.dmpGetFIFOPacketSize();
			dmpReady = true;
		}
		else
		{
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			Serial.print(F("// DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.print(F(" )\n"));
		}
	}

	// robot geometry - received data
	// 20 to 1 geared motor, 3 ticks per motor shaft rotation
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 220.0;
	Geom.EncoderScaler = PI * Geom.wheelDiameter / Geom.ticksPerRevolution;

	Serial.print(F("SUB:Cmd/robot1\n"));		// subscribe only to messages targetted to us
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
			mqRecvBuf[mqIdx++] = '\0';
			StaticJsonBuffer<256> jsonBuffer;
			JsonObject& j = jsonBuffer.parseObject(mqRecvBuf);
			if (strcmp((const char *)j["T"], "Cmd") == 0)
				ProcessCommand(j);
			else
			{
				char t[64];
				sprintf(t, "// rcv <- missing or unrecognized T \"%s\"\n", j["T"]);
				Serial.print(t);
			}
			memset(mqRecvBuf, 0, mqIdx);
			mqIdx = 0;
			return;
		}
		else
			mqRecvBuf[mqIdx++] = c;

		if (mqIdx > sizeof(mqRecvBuf))
		{
			Serial.write("// !! mq buffer overrun\n");
			memset(mqRecvBuf, 0, sizeof(mqRecvBuf));
			mqIdx = 0;
		}
	}
}

bool CalcPose()
{
	bool poseChanged = false;

	uint32_t tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	int32_t delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	// uses DMP for heading
	float headingDelta = (ypr[0] - previousYaw);

	if (abs(RAD_TO_DEG * headingDelta) > .1F)
		poseChanged = true;

	float delta = (delta1 + delta2) * Geom.EncoderScaler / 2.0F;

	if (abs(delta) > .1F)
		poseChanged = true;

	X += delta * sin(H + headingDelta / 2.0F);
	Y += delta * cos(H + headingDelta / 2.0F);

	H += headingDelta;
	if (H < 0)
		H += TWO_PI;
	if (H >= TWO_PI)
		H -= TWO_PI;

	previousYaw = ypr[0];

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;

	return poseChanged;
}

void loop()
{
	// +++ check ultrasonic // pingEventEnabled
	// +++ check status flag / amp draw from mc33926

	if (AhrsEnabled && !dmpReady)
		return; // fail

	if (BumperEventEnabled && digitalReadFast(BUMPER) != lastBumperRead)
	{
		// bumper event may do something to un-bump, so we re-read to set last state
		BumperEvent();
		lastBumperRead = digitalReadFast(BUMPER);
	}

	CheckMq();	// every loop!

	if (escEnabled && (cntr % motorRegulatorFrequency == 0))	// PID regulator
	{
		M1.Tick(); M2.Tick();
	}

	if (escEnabled && (cntr % regulatorFrequency == 0))			// PID regulator
		PilotRegulatorTick();

	// +++ calc pose really needs to be done every dmp interrupt & x ms for encoders
	// or at least integrated
	//  since likelyhood of driving perfectly straight are slim
	//  just using dmp interrupts will probably be ok
	if (cntr % CalcPoseFrequency == 0)
	{
		if (CalcPose())
			if (PoseEventEnabled)
				PublishPose();
	}

	if (cntr % heartbeatEventFrequency == 0)  // heart beat blinky
	{
		digitalWriteFast(LED, !digitalRead(LED));
		if (heartbeatEventEnabled && digitalReadFast(LED))
			PublishHeartbeat();
	}

	if (AhrsEnabled && digitalReadFast(MPU_INT))
	{
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if (mpuIntStatus & 0x10 || fifoCount == 1024) // was if ((mpuIntStatus & 0x10) || fifoCount == 1024)
		{
			mpu.resetFIFO();		// reset so we can continue cleanly
			Serial.print(F("// !!FIFO overflow!!\n"));
		}
		else if (mpuIntStatus & 0x02)
		{
			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			mpu.resetFIFO();		// seems to really help!
		}
	}
	cntr++;
}