//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#ifndef __ArduinoPilot__H
#define __ArduinoPilot__H

// interrupt/phase pins are hardcoded

const int ESC_EN = 12;

#define toggle(X) digitalWrite(X,!digitalRead(X))

typedef struct {
	int ticksPerRevolution;
	float wheelDiameter;
	float wheelBase;
	float EncoderScaler;	// calculated
} Geometry;

double MeanGyroValue();

#endif
