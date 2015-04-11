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

#define Sign(A) (A >= 0 ? 1 : -1)

/// Globals ///////////////////////////////////////////////

const char *Topic = "Topic";

bool AhrsEnabled = true;
bool escEnabled = false;
bool heartbeatEventEnabled = false;
bool BumperEventEnabled = true;
bool DestinationEventEnabled = true;
bool pingEventEnabled = false;
bool PoseEventEnabled = true;

Geometry Geom;
MPU6050 mpu;

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

// FYI
//#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
//#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
//#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// counter based (ie every X cycles)
uint16_t CalcPoseFrequency = 100;		// +++ aim for 20-30 / sec
uint16_t regulatorFrequency = 100;
uint16_t motorRegulatorFrequency = 50;
uint16_t heartbeatEventFrequency = 5000;
uint64_t cntr = 0L;

// +++ change so uses escEnabled instead of -1
// if pins are set to -1, they will not be used, index is the tacho interrupt array
PilotMotor	M1("M1", M1_PWM, M1_DIR, M1_FB, 0, false), 
			M2("M2", M2_PWM, M2_DIR, M2_FB, 1, true);

int  mqIdx = 0;
char mqRecvBuf[128];

/////////////////////////////////////////////////////

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

//////////////////////////////////////////////////
bool dmp_rdy = false;

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

//////////////////////////////////////////////////

void MqLine(char *line, int l)
{
	char t[64];
	const char *T;
	StaticJsonBuffer<256> jsonBuffer;
	JsonObject& j = jsonBuffer.parseObject(line);

	if (strcmp((const char *)j["T"], "Cmd") == 0)
		ProcessCommand(j);
	else
	{
		sprintf_P(t, "// rcv <- missing or unrecognized T \"%s\"\n", j["T"]); Serial.print(t);
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
			mqRecvBuf[mqIdx++] = '\0';
			MqLine(mqRecvBuf, mqIdx);
			memset(mqRecvBuf, 0, mqIdx);
			mqIdx = 0;
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

void BumperEvent()
{

}

//////////////////////////////////////////////////
uint8_t lastBumperRead = 0xff;

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

	// +++ calc pose really needs to be done dmp interrupt or x ms for encoders
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