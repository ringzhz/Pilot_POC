//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#include <Wire.h>
#include <ArduinoJson\ArduinoJson.h>

#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>

#include "digitalWriteFast.h"

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

// common strings
const char *newline = "\n";
const char *value	= "Value";
const char *intvl	= "Int";

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

unsigned long LastPoseTime = 0L;
float previousYaw = 0.0;

bool AhrsEnabled = true;
// escEnabled serves 2 purposes. if it is false at startup, the pins are not initialized
// after startup it is used to actually enable/disable the speed controlers
bool escEnabled = true;
bool heartbeatEventEnabled = false;
bool BumperEventEnabled = true;
bool DestinationEventEnabled = true;
// PoseEventEnabled serves 2 purposes. it is false at startup until AHRS warmup elapses, then
// the pose is reset, a 'AHRD Ready' log message is published and PoseEventEnabled is set to true by default
bool PoseEventEnabled = false;

// counter based (ie every X cycles)
unsigned int CalcPoseFrequency = 600;		// +++ aim for 20-30 / sec
unsigned int pilotRegulatorFrequency = 1200;
unsigned int motorRegulatorFrequency = 400;
unsigned int heartbeatEventFrequency = 5000;
unsigned int checkBumperFrequency = 300;
unsigned int mpuSettledCheckFrequency = 10000;
unsigned long cntr = 0L;

// MPU control/status vars
byte mpuIntStatus;	// holds actual interrupt status byte from MPU
byte devStatus;		// return status after each device operation (0 = success, !0 = error)
byte packetSize;		// expected DMP packet size (default is 42 bytes)
unsigned int fifoCount;			// count of all bytes currently in FIFO
byte fifoBuffer[64];			// FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#include "motor.h"
#include "commands.h"
#include "publish.h"

Geometry Geom;
MPU6050 mpu;

PilotMotor	M1("M1", M1_PWM, M1_DIR, M1_FB, 0, true),
			M2("M2", M2_PWM, M2_DIR, M2_FB, 1, false);

int  mqIdx = 0;
char mqRecvBuf[128];

#define bumperDebounce 3
unsigned short bumperDebounceCntr = 0;
unsigned int lastBumperRead = 0xffff;
long ahrsSettledTime;
bool ahrsSettled = false;

// FYI
//#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
//#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
//#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

////////////////////////////////////////////////////////////

void Log(char *t)
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["T"] = "Log";
	root["Msg"] = t;
	root.printTo(Serial);
	Serial.print(newline);
}

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

	pinMode(LED, OUTPUT);
	pinMode(ESC_ENA, OUTPUT);
	pinMode(BUMPER, INPUT_PULLUP);

	digitalWrite(LED, false);
	digitalWrite(ESC_ENA, false);

	//Serial.print(F("// MotorInit\n"));
	MotorInit();	// interrupt handler(s), pinmode(s)

	if (AhrsEnabled)
	{
		//Serial.print(F("// InitI2C/6050\n"));

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
#endif

		pinMode(MPU_INT, INPUT_PULLUP);

		mpu.initialize();
		delay(20);

		mpu.testConnection();
		devStatus = mpu.dmpInitialize();
		delay(20);

		if (devStatus == 0) 
		{
			mpu.setDMPEnabled(true);
			delay(20);
			mpuIntStatus = mpu.getIntStatus();
			//Serial.print(F("// 6050 rdy\n"));
			packetSize = mpu.dmpGetFIFOPacketSize();
		}
		else
		{
			// ERROR! 1 = initial memory load failed 2 = DMP configuration updates failed
			Serial.print("//! 6050 failed (");
			Serial.print(devStatus);
			Serial.print(")\n");
			BlinkOfDeath(1 + devStatus);
		}

		// +++ supply your own gyro offsets here, scaled for min sensitivity
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

		ahrsSettledTime = millis() + (30 * 1000);
	}

	// +++robot geometry - received data
	// 20 to 1 geared motor, 3 ticks per motor shaft rotation
	Geom.ticksPerRevolution = 60;
	Geom.wheelDiameter = 175.0;
	Geom.wheelBase = 220.0;
	Geom.EncoderScaler = Geom.ticksPerRevolution / (PI * Geom.wheelDiameter);

	Serial.print("// Pilot V2R1.10 (c) spiked3.com\n");
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
			StaticJsonBuffer<128> jsonBuffer;
			JsonObject& j = jsonBuffer.parseObject(mqRecvBuf);
			if (j.containsKey("Cmd"))
				ProcessCommand(j);
			else
				Serial.print("//! Mq.bad.Cmd\n");

			memset(mqRecvBuf, 0, mqIdx);
			mqIdx = 0;
			return;
		}
		else
			mqRecvBuf[mqIdx++] = c;

		if (mqIdx >= sizeof(mqRecvBuf))
			BlinkOfDeath(4);
	}
}

bool CalcPose()
{
	bool poseChanged = false;

	long tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	long delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	// uses DMP for heading
	float headingDelta = (ypr[0] - previousYaw);

	if (abs(RAD_TO_DEG * headingDelta) > .1F)
		poseChanged = true;

	float delta = (delta1 + delta2) * Geom.EncoderScaler / 2.0F;

	if (abs(delta) > .01F)
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

float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousIntegral, float& previousDerivative, float dt)
{
	if (dt <= 0)
		return 0;
	float error = setPoint - presentValue;
	previousIntegral = previousIntegral + error * dt;
	previousDerivative = (error - previousDerivative) / dt;
	return Kp1 * error + Ki1 * previousIntegral + Kd1 * previousDerivative;
}

void loop()
{
	// +++ check status flag / amp draw from mc33926??

	if (cntr % mpuSettledCheckFrequency == 0)
		if (millis() > ahrsSettledTime && !ahrsSettled)
		{
			ahrsSettled = true;
			M1.Reset();
			M2.Reset();
			X = Y = H = 0.0;
			previousYaw = ypr[0];	// base value for heading 0
			PoseEventEnabled = true;
			Log("AHRS Ready");
		}

	CheckMq();

	// +++ note - v2r1 schematic is wrong - jumper should be tied to ground not vcc
	//  so for now use outside bumper pin and grnd (available on outside reset jumper) for bumper @v2r1
	// +++ im not convinced, the intent was to use normally closed, and it should be ok as is?
	if (BumperEventEnabled && cntr & checkBumperFrequency == 0)
	{
		if (bumperDebounceCntr == 0)
		{
			int thisBumper = digitalReadFast(BUMPER);
			if (thisBumper != lastBumperRead)
			{
				// new (debounced) event
				lastBumperRead = thisBumper;
				bumperDebounceCntr = bumperDebounce;
				BumperEvent(thisBumper == 0);
			}
		}
		else
			if (--bumperDebounceCntr < 0)
				bumperDebounceCntr = 0;
	}

	if (escEnabled && (cntr % motorRegulatorFrequency == 0))
	{
		M1.Tick();
		M2.Tick();
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

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow, this happens on occasion
		if (mpuIntStatus & 0x10 || fifoCount == 1024) 
		{
			Serial.print("//!mpuOvf\n");
			mpu.resetFIFO();
		}
		else if (mpuIntStatus & 0x02)
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			mpu.resetFIFO();
		}
	}
	cntr++;
}

