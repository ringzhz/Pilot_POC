//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#pragma once

#include <ArduinoJson.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>

#include "digitalWriteFast.h"		// locally modified copy

// pins are defines to allow feastRead/Writes
// interrupt/phase pins are hardcoded in motor.cpp
#define BUMPER	4
#define LED		13
#define ESC_ENA 12
#define MPU_INT 7

// motor pins
#define M1_PWM	5
#define M2_PWM	6
#define M1_DIR	11
#define M2_DIR	17	// A3
#define M1_FB	16	// A2
#define M2_FB	15	// A1

// A0 was reserved for the buttons on prototype board, but is no longer used - so it is available

#define AHRS_SETTLE_TIME 30	// seconds

// common strings
char *value = "Value";
char *intvl = "Int";
#define LOG "// "
#define ERROR "//!"

#define DBGP(t) Serial.print(LOG t)
#define DBGV(t,v) Serial.print(" " t "="); Serial.print(v)
#define DBGE() Serial.println()

////////////////////////////////////////////////////////////

typedef struct {
	int ticksPerMeter;
	int mMax;
} Geometry;

float X = 0.0;		// internally mm, published in meters
float Y = 0.0;
float H = 0.0;		// internally using radians, published in degrees

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} pidData;

float previousYaw = 0.0;

bool AhrsEnabled = true;
// escEnabled serves 2 purposes. if it is false at startup, the pins are not initialized
// after startup it is used to actually enable/disable the speed controlers
bool escEnabled = true;
bool heartbeatEventEnabled = false;
bool BumperEventEnabled = true;
bool DestinationEventEnabled = true;
// PoseEventEnabled serves 2 purposes. it is false at startup until AHRS warmup elapses, then
// the pose is reset, an move completeevent is published and PoseEventEnabled is set to true
bool PoseEventEnabled = false;

// counter based (ie every X loops)
unsigned int CalcPoseFrequency = 50;
unsigned int pilotRegulatorFrequency = 20;
unsigned int heartbeatEventFrequency = 2000;
unsigned int checkBumperFrequency = 300;
unsigned int mpuSettledCheckFrequency = 10000;
unsigned long cntr = 0L;

// MPU control/status vars
byte mpuIntStatus;		// holds actual interrupt status byte from MPU
byte packetSize;		// expected DMP packet size (default is 42 bytes)
byte devStatus;			// return status after each device operation (0 = success, !0 = error)
unsigned int fifoCount;	// count of all bytes currently in FIFO
byte fifoBuffer[64];	// FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

#include "motor.h"
#include "commands.h"
#include "publish.h"

Geometry Geom;
MPU6050 mpu;

PilotMotor	M1(M1_PWM, M1_DIR, M1_FB, 0, true), M2(M2_PWM, M2_DIR, M2_FB, 1, false);

int  mqIdx = 0;
char mqRecvBuf[128];

#define bumperDebounce 3
byte bumperDebounceCntr = 0;
unsigned int lastBumperRead = 0xffff;
long ahrsSettledTime;
bool ahrsSettled = false;

////////////////////////////////////////////////////////////

void BlinkOfDeath(int code)
{
	while (1)
	{
		for (int i = 0; i < code; i++)
		{
			digitalWrite(LED, HIGH);
			delay(200);
			digitalWrite(LED, LOW);
			delay(200);
		}
		delay(600);
	}
}

void setup()
{
	Serial.begin(115200);
	delay(20);

	pinMode(LED, OUTPUT);
	pinMode(ESC_ENA, OUTPUT);
	pinMode(BUMPER, INPUT_PULLUP);

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

		mpu.initialize();
		delay(10);

		mpu.testConnection();
		devStatus = mpu.dmpInitialize();
		delay(10);

		if (devStatus == 0)
		{
			mpu.setDMPEnabled(true);
			delay(10);
			mpuIntStatus = mpu.getIntStatus();
			packetSize = mpu.dmpGetFIFOPacketSize();
		}
		else
		{
			// ERROR! 1 = initial memory load failed 2 = DMP configuration updates failed
			Serial.print(ERROR "6050 failed (");
			Serial.print(devStatus);
			Serial.println(")");
			BlinkOfDeath(1 + devStatus);
		}

		// defaults from example is good default starting point
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788);

		ahrsSettledTime = millis() + (AHRS_SETTLE_TIME * 1000);
	}

	Serial.println(LOG "S3 Pilot V0.6.12 (c) 2015 spiked3.com");
}

void CheckMq()
{
	while (Serial.available())
	{
		char c = Serial.read();
		if (c == '\n')		// end of line, process
		{
			mqRecvBuf[mqIdx++] = '\0';
			StaticJsonBuffer<128> jsonBuffer;
			JsonObject& j = jsonBuffer.parseObject(mqRecvBuf);
			if (j.containsKey("Cmd"))
				ProcessCommand(j);
			else
				Serial.println(ERROR "NoCmd");
			mqIdx = 0;
			return;
		}
		else
			mqRecvBuf[mqIdx++] = c;

		if (mqIdx >= sizeof(mqRecvBuf))
			BlinkOfDeath(4);
	}
}

float NormalizeHeading(float& h, float min, float max)
{
	while (h > max)
		h -= TWO_PI;
	while (H < min)
		h += TWO_PI;
	return h;
}

bool CalcPose()
{
	bool poseChanged = false;

	long tachoNow1 = M1.GetRawTacho(),
		tachoNow2 = M2.GetRawTacho();

	float delta1 = (tachoNow1 - M1.lastPoseTacho) * 1000 / Geom.ticksPerMeter,
		delta2 = (tachoNow2 - M2.lastPoseTacho) * 1000 / Geom.ticksPerMeter;

	// uses DMP for heading
	float headingDelta = (ypr[0] - previousYaw);

	if (abs(RAD_TO_DEG * headingDelta) > .1F)
		poseChanged = true;

	float delta = (delta1 + delta2) / 2;

	if (abs(delta) > 1)
		poseChanged = true;

	X += delta * sin(H + headingDelta / 2);
	Y += delta * cos(H + headingDelta / 2);

	H += headingDelta;
	NormalizeHeading(H, -PI, PI);

	previousYaw = ypr[0];

	M1.lastPoseTacho = tachoNow1;
	M2.lastPoseTacho = tachoNow2;

	return poseChanged;
}

void loop()
{
	CheckMq();

	// +++ check status flag / amp draw from mc33926??

	if (!ahrsSettled && (cntr % mpuSettledCheckFrequency == 0))
		if (millis() > ahrsSettledTime)
		{
			ahrsSettled = true;
			//M1.Reset();
			//M2.Reset();
			//X = Y = H = 0.0;
			//previousYaw = ypr[0];	// base value for heading 0
			PoseEventEnabled = true;
			MoveCompleteEvent(true);
		}

	// +++ note - v2r1 schematic is wrong - jumper should be tied to ground not vcc
	//  so for now use outside bumper pin and grnd (available on outside reset jumper) for bumper @v2r1
	// +++ im not convinced it is wrong, the intent was to use normally closed and trigger on open, and it should be ok as is (need to flip sign is all)?
	if (BumperEventEnabled && (cntr % checkBumperFrequency == 0))
	{
		if (bumperDebounceCntr == 0)
		{
			int thisBumper = digitalReadFast(BUMPER);
			if (thisBumper != lastBumperRead)
			{
				// new (debounced) event
				lastBumperRead = thisBumper;
				bumperDebounceCntr = bumperDebounce;
				BumperEvent(thisBumper != 0);
			}
		}
		else
			if (--bumperDebounceCntr < 0)
				bumperDebounceCntr = 0;
	}

	if (escEnabled && (cntr % pilotRegulatorFrequency == 0))
		PilotRegulatorTick();

	if (cntr % CalcPoseFrequency == 0)
		if (CalcPose())
			if (PoseEventEnabled)
				PublishPose();

	if (cntr % heartbeatEventFrequency == 0)  // heart beat blinky
	{
		digitalWriteFast(LED, !digitalRead(LED));
		if (heartbeatEventEnabled && digitalReadFast(LED))
			PublishHeartbeat();
	}

	if (AhrsEnabled && digitalReadFast(MPU_INT))
	{
		mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount();

		// check for overflow, this happens on occasion
		if (mpuIntStatus & 0x10 || fifoCount == 1024)
			Serial.println(ERROR "mpuOvf");

		if (mpuIntStatus & 0x02)
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		}

		mpu.resetFIFO();
	}
	cntr++;
}

