//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved


// 6/11/15 note, I set the AHRS back to default rate, i cant remember if it made a difference or not. I dont think it did?

#define NOLIMIT 0x7fffffff

#define KICK_POWER 60
#define VELOCITY_SAMPLE_RATE 10

unsigned long lastTickTime;

volatile long rawTacho[2];		// interrupt 0 & 1 tachometers

extern Geometry Geom;

pidData MotorPID{
	.3, .9, 0.0025	// seems pretty good on 06/19/2015
};


float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& previousIntegral, float& previousDerivative, float dt);

// a motor is generally not intended to be accessed directly, 
//  but by the pilot regulator who also controls heading

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
	float power;
	float previousError, integral, derivative;
	float timeAccumulator = 0; // used to calculate velocity at @ VELOCITY_SAMPLE_RATE
	int velocitySampleCtr = 0;
	float tgtVelocity;		// is what we asked for
	float velocity;			// is what we have 

public:
	PilotMotor(unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd);
	void Reset();
	void SetSpeed(float speed, int acceleration);
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
	SetSpeed(0, 0);
	rawTacho[interruptIndex] = lastTickTacho = lastPoseTacho = 0L;
}

void PilotMotor::PinPower(int p)
{
	unsigned short newDir = LOW;
	int realPower = 0;

	if (p != lastPinPower)
	{
		//DBGP("PinPower");  DBGV("Pin", pwmPin);  DBGV("", p); DBGE();
		newDir = (p >= 0) ? (reversed ? HIGH : LOW) : (reversed ? LOW : HIGH);
		realPower = map(abs(p), 0, 100, 0, 255);
		digitalWrite(dirPin, newDir);
		analogWrite(pwmPin, realPower);
		lastPinPower = p;
	}
}

void PilotMotor::SetSpeed(float setSpeed, int setAccel)
{
	tgtVelocity = setSpeed * Geom.mMax / 100;	// speed as % times max ticks speed
	//DBGP("SetSpeed");  DBGV("tgtVelocity", tgtVelocity); DBGE();
	if (setSpeed == 0)
		previousError = derivative = integral = 0;
}

void PilotMotor::Tick(unsigned int eleapsedMs)
{
	timeAccumulator += eleapsedMs;
	if (velocitySampleCtr++ == VELOCITY_SAMPLE_RATE)	// provides an average over X samples to smooth digital aliasing
	{
		float newVelocity = (tickTacho - lastTickTacho) * 1000 / timeAccumulator;	// TPS
		velocity = (.90 * newVelocity) + (.10 * velocity);
		timeAccumulator = 0;
		velocitySampleCtr = 0;
		lastTickTacho = tickTacho;
	}

	// a breaking PID would be better
	// as is, regular drive pid is too violent for breaking
	if (tgtVelocity == 0)
		power = previousError = integral = derivative = 0;

	// !!! fix suggested by jesseg - use pid output as power, not an adjustment
	// it finally acts like expected, thanks jesseg !!!
	float pid = Pid(tgtVelocity, velocity, MotorPID.Kp, MotorPID.Ki, MotorPID.Kd,
		previousError, integral, derivative, (float) eleapsedMs / 1000);

	power = constrain(pid, -100, 100);
}

//----------------------------------------------------------------------------

float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& integral, float& derivative, float dt)
{
	float error = setPoint - presentValue;
	integral = integral + error * dt;
	float newDerivative = (error - previousError) / dt; 
	derivative = (.25 * newDerivative) + (.75 * derivative);
	float output = Kp * error + Ki * integral + Kd * derivative;	
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

	escEnabled = false;
}

float Distance(float x1, float y1, float x2, float y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void MoveCompleteEvent(bool success);
bool headingStop = false;
bool moveStop = false;
float headingGoal = 0;

float AngleBetween(float angle1, float angle2)
{
	while (angle1 < 0) angle1 += TWO_PI;
	while (angle2 < 0) angle2 += TWO_PI;
	while (angle1 > TWO_PI) angle1 -= TWO_PI;
	while (angle2 > TWO_PI) angle2 -= TWO_PI;

	float diff = angle1 - angle2;
	float absDiff = abs(diff);

	if (absDiff <= PI)
		return absDiff == PI ? PI : diff;	// so -180 is 180 (in radians)

	else if (angle1 > angle2)
		return absDiff - TWO_PI;

	return TWO_PI - absDiff; 
}

void PilotRegulatorTick()
{
	unsigned long now = millis();
	unsigned int tickElapsedTime = now - lastTickTime;

	if (headingStop)
	{
		float absDiff = abs(AngleBetween(H, headingGoal));
		if (absDiff < (5 * DEG_TO_RAD))
		{
			M1.tgtVelocity = M2.tgtVelocity = 0;
			headingStop = false;
			MoveCompleteEvent(true);
		}
		else if (absDiff < (20 * DEG_TO_RAD))
		{
			M1.power *= .5;		// +++ is this working?
			M2.power *= .5;
		}
	}

	M1.tickTacho = M1.GetRawTacho();
	M2.tickTacho = M2.GetRawTacho();

	M1.Tick(tickElapsedTime);
	M2.Tick(tickElapsedTime);

	M1.PinPower(M1.power);
	M2.PinPower(M2.power);

	lastTickTime = now;
}
