//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#define NOLIMIT 0x7fffffff

// pilot (outer) regulator (uses Pid[PILOT_PID])
float lastHeadaing = 0, tgtHeading;
float travelX = 0, travelY = 0;
unsigned long lastTickTime;
float previousError, previousIntegral, previousDerivative;
bool Traveling = false;
bool Rotating = false;

volatile long rawTacho[2];		// interrupt 0 & 1 tachometers

extern Geometry Geom;
extern pidData PidTable[];

float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& previousIntegral, float& previousDerivative, float dt);

// a motor is generally not accessed directly, but by the pilot regulator who also controls heading

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

ISR(MotorISR1)
{
	int b = digitalReadFast(8);
	if (digitalReadFast(2))
		b ? rawTacho[0]-- : rawTacho[0]++;
	else
		b ? rawTacho[0]++ : rawTacho[0]--;
}

ISR(MotorISR2)
{
	int b = digitalReadFast(9);
	if (digitalReadFast(3))
		b ? rawTacho[1]-- : rawTacho[1]++;
	else
		b ? rawTacho[1]++ : rawTacho[1]--;
}

PilotMotor::PilotMotor(unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd)
{
	pwmPin = pwm;
	dirPin = dir;
	feedBackPin = fb;
	interruptIndex = idx;
	reversed = revrsd;

	if (escEnabled)
	{
		pinMode(pwm, OUTPUT);
		pinMode(dir, OUTPUT);
		Reset();
	}
}

void PilotMotor::Reset()
{
	SetSpeed(0, 0, NOLIMIT);	
	rawTacho[interruptIndex] = lastTickTacho = lastPoseTacho = 0L;
	previousError = previousDerivative = previousIntegral = 0;	
	lastTickTime = millis();
}

void PilotMotor::PinPower(int p)
{
	unsigned short newDir = LOW;
	int realPower = 0;

	if (p != lastPinPower)
	{
		//DBGV(LOG "PinPwr p=", p);
		newDir = (p >= 0) ? (reversed ? HIGH : LOW) : (reversed ? LOW : HIGH);
		realPower = map(abs(p), 0, 100, 0, 255);
		digitalWrite(dirPin, newDir);
		analogWrite(pwmPin, realPower);
		lastPinPower = p;
	}
}

// limit actually sets the direction, use +NOLIMIT/-NOLIMIT for continuous
void PilotMotor::SetSpeed(float setSpeed, int setAccel, long setLimit)
{
	tgtVelocity = setSpeed * Geom.MMax / 100;	// speed as % times max ticks speed
	limit = setLimit;
	checkLimit = abs(setLimit) != NOLIMIT;
	moving = tgtVelocity != 0;
	previousError = previousDerivative = previousIntegral = 0;

	if (setSpeed != 0 && velocity == 0)
		PinPower(setSpeed >= 0 ? 40 : -40);		// minimum kick to get motor moving
}

void PilotMotor::Stop(bool immediate)
{
	SetSpeed(0, 0, NOLIMIT);
}

// prevent runaways
#define iMax 5
#define dMax 5

void PilotMotor::Tick(unsigned int eleapsedMs)
{
	velocity = (tickTacho - lastTickTacho) * 1000 / eleapsedMs;	// TPS
	float pid = 0;

	if (moving)
		pid = Pid(tgtVelocity, velocity, PidTable[MOTOR_PID].Kp, PidTable[MOTOR_PID].Ki, PidTable[MOTOR_PID].Kd, 
			previousError, previousIntegral, previousDerivative, eleapsedMs / 1000);
	if (tgtVelocity != 0)
		power = constrain(power + pid, -100, 100);
	else
		power = 0;
	lastTickTacho = tickTacho;
}

//----------------------------------------------------------------------------

float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& previousIntegral, float& previousDerivative, float dt)
{
	float error = setPoint - presentValue;
	float integral = constrain((previousIntegral + error) * dt, -iMax, iMax);
	float derivative = constrain((previousDerivative - error) * dt, -dMax, dMax);
	float output = Kp * error + Ki * integral + Kd * derivative;
	previousIntegral = integral;
	previousDerivative = derivative;
	previousError = error;
	return output;
}

void MotorInit()
{
	rawTacho[0] = rawTacho[1] = 0L;

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

	lastHeadaing = tgtHeading = travelX = travelY = 0;
	previousError = previousIntegral = previousDerivative = 0;

	escEnabled = false;
}

float Distance(float x1, float y1, float x2, float y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2* y1));
}

void Travel(float x, float y, int speed)
{
	travelX = x;
	travelY = y;
	previousError = previousIntegral = previousDerivative = 0;
	Traveling = true;
	M1.SetSpeed(speed, 0, +NOLIMIT);
	M2.SetSpeed(speed, 0, +NOLIMIT);
}

void Travel(float distance, int speed)
{
	Travel(X + (distance * cos(H)), Y - (distance * sin(Y)), speed);
}

void PilotRegulatorTick()
{
	unsigned long now = millis();
	unsigned int tickElapsedTime = now - lastTickTime;

	float adjustment = 0;

	// +++ pilot regulation not tested in any way, waiting chassis

	M1.tickTacho = M1.GetRawTacho();
	M2.tickTacho = M2.GetRawTacho();

	M1.Tick(tickElapsedTime);
	M2.Tick(tickElapsedTime);

	if (Traveling)
	{
		float headingTo = atan2(travelY - Y, travelX - X);
		while (H > PI)
			H -= TWO_PI;
		while (H < -PI)
			H += TWO_PI;

		adjustment = Pid(headingTo, H, PidTable[PILOT_PID].Kp, PidTable[PILOT_PID].Ki, PidTable[PILOT_PID].Kd, 
			previousError, previousIntegral, previousDerivative, tickElapsedTime);

		// +++ sanity check?

		if (Distance(travelX, X, travelY, Y) < .1)
		{
			Traveling = false;
			adjustment = M1.power = M2.power = 0;
		}
	}
	else if (Rotating)
	{
		if (abs(tgtHeading - H) < (DEG_TO_RAD * 2))
			Rotating = false;
		else
			adjustment = Pid(tgtHeading, H, PidTable[PILOT_PID].Kp, PidTable[PILOT_PID].Ki, PidTable[PILOT_PID].Kd, 
				previousError, previousIntegral, previousDerivative, tickElapsedTime);

		M1.power = M2.power = 0;	// always rotate in place
	}

	// +++this is the reason we need to normalize -180/+180, not 0-360

#if 1
        if (adjustment >= 0)
	{
		M1.power += abs(adjustment)*10;
		M2.power -= abs(adjustment)*10;
	}
	else
	{
		M1.power -= abs(adjustment)*10;
		M2.power += abs(adjustment)*10;
	}

#endif
	M1.PinPower(M1.power);
	M2.PinPower(M2.power);
	lastTickTime = now;
}


