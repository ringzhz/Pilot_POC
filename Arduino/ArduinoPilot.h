#ifndef __ArduinoPilot__H
#define __ArduinoPilot__H

// interrupt/phase pins are hardcoded

const int ESC_EN = 12;

#define toggle(X) digitalWrite(X,!digitalRead(X))

typedef struct {
	int ticksPerRevolution;
	float wheelDiameter;
	float wheelBase;
	float EncoderScalar;	// calculated
} Geometry;

#endif
