#ifndef __ArduinoPilot__H
#define __ArduinoPilot__H

// digital pins
const int LED = 13;

// motor pins
const int M1_PWM = 5;
const int M2_PWM = 6;
const int M1_DIR = 11;
const int M2_DIR = 10;

/*  hardcoded in PilotMotor!
#define M1_A 2	// interrupt 0 = pin 2
#define M1_B 8
#define M2_A 3	// interrupt 1 = pin 3
#define M2_B 9
*/

const int ESC_EN = 12;

#define toggle(X) digitalWrite(X,!digitalRead(X))

// analog pins
const int M1_FB = 0;
const int M2_FB = 1;

//#define SDA 4
//#define SCL 5

typedef struct {
	int ticksPerRevolution;
	float wheelDiameter;
	float wheelBase;
	float ticksToMM;	// calculated
} Geometry;

#endif
