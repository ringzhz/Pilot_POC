//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#define NOLIMIT 0x7fffffff

extern Geometry Geom;
extern pidData PidTable[];
extern volatile long rawTacho[2];

class PilotMotor
{
public:
	// pin level
	byte pwmPin;
	byte dirPin;
	byte feedBackPin;
	byte interruptIndex;
	bool reversed;
	float lastPinPower;

	// used by pose
	long lastPoseTacho;

	// used by motor regulator (tick)
	long tickTacho, lastTickTacho;

	// regulator variables
	// speed/velocity are ticks per second
	bool moving;
	bool checkLimit;
	long limit;
	float power;
	float previousError, previousIntegral, previousDerivative;
	float tgtVelocity;		// is what we asked for
	float velocity;			// is what we have 

public:
	PilotMotor(unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd);
	void Reset();
	void SetSpeed(float speed, int acceleration, long limit);
	void Stop(bool sudden);
	void Tick(unsigned int elapsed);

protected:
	friend void PilotRegulatorTick();
	friend bool CalcPose();
	friend void PublishHeartbeat();
	long GetRawTacho() { return reversed ? -rawTacho[interruptIndex] : rawTacho[interruptIndex]; }
	void PinPower(int power);
};

extern PilotMotor M1, M2;

float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& previousIntegral, float& previousDerivative, float dt);
void MotorInit();
float Distance(float x1, float y1, float x2, float y2);
void Travel(float x, float y, int speed);
void Travel(float distance, int speed);
float MapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void PilotRegulatorTick();
