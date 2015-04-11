//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

// interrupt/phase pins are hardcoded

extern const int ESC_EN;

#define toggle(X) digitalWrite(X,!digitalRead(X))

typedef struct {
	int ticksPerRevolution;
	float wheelDiameter;
	float wheelBase;
	float EncoderScaler;	// calculated
} Geometry;

extern const char *Topic;

extern bool AhrsEnabled;
extern bool escEnabled;
extern bool heartbeatEventEnabled;
extern bool BumperEventEnabled;
extern bool DestinationEventEnabled;
extern bool pingEventEnabled;

extern uint16_t checkMqFrequency;			// we check one byte at a time, so do often
extern uint16_t CalcPoseFrequency;		// +++ aim for 20-30 / sec
extern uint16_t regulatorFrequency;
extern uint16_t heartbeatEventFrequency;


extern MPU6050 mpu;
extern uint8_t fifoBuffer[64];

extern Quaternion q;
extern VectorFloat gravity;
extern float euler[3];
extern float ypr[3];

extern PilotMotor M1, M2;
extern Geometry Geom;

void Log(String t);

// pins are defines to allow feastRead/Writes

#define BUMPER 4

#define LED 13

#define ESC_EN 12

// motor pins
#define M1_PWM 5
#define M2_PWM 6
#define M1_DIR 11
#define M2_DIR 17	// A3
#define M1_FB  16	// A2
#define M2_FB  15	// A1

