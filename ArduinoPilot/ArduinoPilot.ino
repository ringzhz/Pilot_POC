//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright � 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include <Wire.h>
#include <ArduinoJson.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>

#include "digitalWriteFast.h"

#ifdef DEBUG
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

#define toggle(X) digitalWrite(X,!digitalRead(X))

#define Sign(A) (A >= 0 ? 1 : -1)

////////////////////////////////////////////////////////////

typedef struct {
	int ticksPerRevolution;
	float wheelDiameter;
	float wheelBase;
	float EncoderScaler;	// calculated
} Geometry;

struct CmdFunction
{
	const char *cmd;
	bool(*f)(JsonObject&  j);
};

float X = 0.0;		// internally mm, broadcast in meters
float Y = 0.0;
float H = 0.0;		// internally using radians, broadcasts in degrees

uint64_t LastPoseTime = 0L;
float previousHeading = 0.0;

class PilotMotor
{
public:
	char motorName[3];
	char safetyEOS = '\0';
	uint8_t pwmPin;
	uint8_t dirPin;
	uint8_t feedBackPin;
	uint8_t interruptIndex;
	bool reversed;

	uint32_t lastUpdateTime;
	uint32_t lastTacho;
	float desiredSpeed;
	float actualSpeed;

	float lastPower;
	float previousError;
	float previousIntegral;

	PilotMotor(const char *name, uint8_t pwm, uint8_t dir, uint8_t fb, uint8_t idx, bool revrsd);
	void Reset();
	uint32_t GetTacho();
	void SetSpeed(int spd);

	void Tick();
};

uint8_t MotorMax = 100;
float Kp1, Ki1, Kd1;		// per motor regulator
float Kp2, Ki2, Kd2;		// synchronizing regulator

volatile uint32_t tacho[2];		// interrupt 0 & 1 tachometers

/// Globals ///////////////////////////////////////////////

const char *Topic = "Topic";

bool AhrsEnabled = true;
bool escEnabled = true;
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

bool dmp_rdy = false;

/////////////////////////////////////////////////////

ISR(MotorISR1)
{
	int b = digitalReadFast(8);
	if (digitalReadFast(2))
		b ? tacho[0]++ : tacho[0]--;
	else
		b ? tacho[0]-- : tacho[0]++;
}

ISR(MotorISR2)
{
	int b = digitalReadFast(9);
	if (digitalReadFast(3))
		b ? tacho[1]++ : tacho[1]--;
	else
		b ? tacho[1]-- : tacho[1]++;
}

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

void MotorInit()
{
	Serial.print("// MotorInit ... \n");

	tacho[0] = tacho[1] = 0L;

	// !! hardcoded interrupt handlers !!
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);

	attachInterrupt(PCINT0, MotorISR1, CHANGE);	// pin 2
	attachInterrupt(PCINT1, MotorISR2, CHANGE); // pin 3

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
}

void PilotRegulatorTick()
{
	// +++ regulation needed now! but waiting on hardware :|
}


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

void PublishPose()
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Pose";
	root["X"].set(X / 1000, 4);		// mm to meter
	root["Y"].set(Y / 1000, 4);
	root["H"].set(RAD_TO_DEG * H, 2);
	root.printTo(Serial); Serial.print('\n');
}

void PublishHeartbeat()
{
	StaticJsonBuffer<128> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[Topic] = "robot1";
	root["T"] = "Heartbeat";

#if 1
	root["M1Tach"].set(M1.GetTacho(), 0);  // 0 is the number of decimals to print
	root["M2Tach"].set(M2.GetTacho(), 0);
	if (AhrsEnabled)
	{
		root["Yaw"].set(ypr[0], 4);
		root["Pit"].set(ypr[1], 4);
		root["Rol"].set(ypr[2], 4);
	}
#endif

	root.printTo(Serial); Serial.print('\n');
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

bool CalcPose()
{
	bool poseChanged = false;

	uint32_t tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	int32_t delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	// uses DMP for heading
	float headingDelta = (ypr[0] - previousHeading);

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

	previousHeading = ypr[0];

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;

	return poseChanged;
}

//////////////////////////////////////////////////
uint8_t lastBumperRead = 0xff;

bool cmdTest1(JsonObject&  j)
{
	DBGLOG(F("::cmdTest1"));
	Serial.print(F("// cmdTest1\n"));
	return true;
}

//////////////////////////////////////////////////

bool cmdMmax(JsonObject&  j)
{
	DBGLOG(F("::cmdMmax"));
	MotorMax = j["Value"];
	return true;
}

bool cmdPid1(JsonObject&  j)
{
	DBGLOG(F("::cmdPid1"));
	Kp1 = j["P"];
	Ki1 = j["I"];
	Kd1 = j["D"];
	return true;
}

bool cmdBump(JsonObject&  j)
{
	DBGLOG(F("::cmdBump"));
	BumperEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdDest(JsonObject&  j)
{
	DBGLOG(F("::cmdDest"));
	DestinationEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdHeartbeat(JsonObject&  j)
{
	DBGLOG(F("::cmdHeartbeat"));
	heartbeatEventEnabled = j["Value"] == 1;
	if (j.containsKey("Int"))
		heartbeatEventFrequency = j["Int"];
	return true;
}

bool cmdPing(JsonObject&  j)
{
	DBGLOG(F("::cmdPing"));
	pingEventEnabled = j["Value"] == 1;
	return true;
}

bool cmdReset(JsonObject&  j)
{
	// by including specific variables, you can set pose to a particular value
	DBGLOG(F("::cmdReset"));
	M1.Reset();
	M2.Reset();
	X = Y = H = previousHeading = 0.0;
	if (j.containsKey("X"))
		X = j["X"];
	if (j.containsKey("Y"))
		Y = j["Y"];
	if (j.containsKey("H"))
		H = DEG_TO_RAD * (float)j["H"];

	previousHeading = ypr[0];	// base value

	return true;
}

bool cmdEsc(JsonObject&  j)
{
	DBGLOG(F("::cmdEsc"));
	escEnabled = j["Value"] == 1;
	digitalWriteFast(ESC_ENA, escEnabled);
	return true;
}

bool cmdPose(JsonObject&  j)
{
	DBGLOG(F("::cmdPose"));
	PoseEventEnabled = j["Value"] == 1;
	if (j.containsKey("Int"))
		CalcPoseFrequency = j["Int"];
	return true;
}

bool cmdGeom(JsonObject&  j)
{
	// +++
	DBGLOG(F("::cmdGeom"));
	return false;
}

bool cmdPower(JsonObject&  j)
{
	// +++ actually more of a testing function, will probably go away
	//char t[16];
	DBGLOG(F("::cmdPower"));

	if (escEnabled)
	{
		int p = constrain((int)j["Value"], -100, 100);
		//sprintf(t, "// P %d\n", p); Serial.print(t);
		M1.SetSpeed(p);
		M2.SetSpeed(p);
	}

	return true;
}

bool cmdMove(JsonObject&  j)
{
	DBGLOG(F("::cmdMove"));
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

bool cmdRot(JsonObject&  j)
{
	DBGLOG(F("::cmdRot"));
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

bool cmdGoto(JsonObject&  j)
{
	DBGLOG(F("::cmdGoto"));
	float speed = (int)j["Speed"] / 10.0F;

	return false;
}

////////////////////////////////////////////////////

CmdFunction cmdTable[] {
	{ "Test1", cmdTest1 },
	{ "Reset", cmdReset },
	{ "Geom", cmdGeom },
	{ "MMax", cmdMmax },
	{ "PID1", cmdPid1 },
	{ "Esc", cmdEsc },
	{ "Rot", cmdRot, },
	{ "GoTo", cmdGoto, },
	{ "Move", cmdMove },
	{ "Bump", cmdBump, },
	{ "Dest", cmdDest, },
	{ "Heartbeat", cmdHeartbeat, },
	{ "Pose", cmdPose, },
	{ "Power", cmdPower, },
	{ "Ping", cmdPing, },
};

void ProcessCommand(JsonObject& j)
{
	for (int i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
		if (strcmp(cmdTable[i].cmd, (const char *)j["Cmd"]) == 0)
		{
			bool rc = (*cmdTable[i].f)(j);
			break;
		}
}

PilotMotor::PilotMotor(const char *name, uint8_t pwm, uint8_t dir, uint8_t fb, uint8_t idx, bool revrsd)
{
	strncpy(motorName, name, sizeof(motorName));
	pwmPin = pwm;
	dirPin = dir;
	feedBackPin = fb;
	interruptIndex = idx;
	reversed = revrsd;

	if (escEnabled)
	{
		pinMode(pwm, OUTPUT);
		pinMode(dir, OUTPUT);
	}

	Reset();
}

void PilotMotor::Reset()
{
	tacho[interruptIndex] = lastTacho = 0L;
	lastPower = desiredSpeed = actualSpeed = 0.0;
	previousError = previousIntegral = 0.0;
	//lastUpdateTime = ? ? ? ;
}

uint32_t PilotMotor::GetTacho()
{
	return reversed ? -tacho[interruptIndex] : tacho[interruptIndex];
}

void PilotMotor::SetSpeed(int spd)
{
	char t[64];
	// speed is a +/- percent of max	
	uint8_t newDir = (spd >= 0) ? (reversed ? 1 : 0) : (reversed ? 0 : 1);
	int16_t newSpeed = map(abs(spd), 0, 100, 0, 255 * MotorMax / 100);
	digitalWrite(dirPin, newDir);
	analogWrite(pwmPin, newSpeed);
	desiredSpeed = newSpeed;
	sprintf(t, "// %d,%d >> %s\n", newDir, newSpeed, motorName); Serial.print(t);
}

void PilotMotor::Tick()
{

}

///////////////////////////////////////////////////

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