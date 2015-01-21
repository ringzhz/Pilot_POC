#ifndef __ArduinoPilot__H
#define __ArduinoPilot__H

// Since arduino doesnt really do OO well, its silly to try and force classes
typedef struct {
	byte pwmPin;
	byte dirPin;
	byte intPin;
	byte bPin;
	bool reverse;
	long lastUpdateTime;
	long tacho;
	float desiredSpeed;		// radians per sec
	float actualSpeed;
	bool motorCW = true;
	float power;
	float previousError;
	float previousIntegral;
} Motor;

#endif
