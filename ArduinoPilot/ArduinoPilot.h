//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#ifndef __ArduinoPilot__H
#define __ArduinoPilot__H

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
extern bool heartbeatEnabled;

extern MPU6050 mpu;
extern uint8_t fifoBuffer[64];

extern Quaternion q;
extern VectorFloat gravity;
extern float euler[3];
extern float ypr[3];

extern PilotMotor M1, M2;
extern Geometry Geom;
;

#endif
