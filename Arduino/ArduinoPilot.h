#ifndef __ArduinoPilot__H
#define __ArduinoPilot__H

// digital pins

#define LED 13

// motor pins
#define M1_PWM 5
#define M2_PWM 6
#define M1_DIR 11
#define M2_DIR 10

#define M1_A 2	// interrupt 0 = pin 2
#define M1_B 8
#define M2_A 3	// interrupt 1 = pin 3
#define M2_B 9

#define ESC_EN 12

// analog pins
#define M1_FB 0
#define M2_FB 1

//#define SDA 4
//#define SCL 5


// Since arduino doesnt really do OO well, its silly to try and force classes
typedef struct {
	int pwmPin;
	int dirPin;
	int intPin;
	int bPin;
	bool reverse;
	long lastUpdateTime;
	volatile long tacho;
	volatile long lastTacho;
	float desiredSpeed;		// radians per sec
	float actualSpeed;
	bool motorCW = true;
	float power;
	float previousError;
	float previousIntegral;
} Motor;

typedef struct {
	int ticksPerRevolution;
	float wheelDiamter;
	float wheelBase;
} Geometry;

#endif
