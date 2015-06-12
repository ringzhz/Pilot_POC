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
	.01, 4.0, 4.0	// seems pretty good on 05/20/2015
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

// limit actually sets the direction, use +NOLIMIT/-NOLIMIT for continuous (at least that is the future intention)
void PilotMotor::SetSpeed(float setSpeed, int setAccel)
{
	tgtVelocity = setSpeed * Geom.mMax / 100;	// speed as % times max ticks speed
	//DBGP("SetSpeed");  DBGV("tgtVelocity", tgtVelocity); DBGE();
	if (setSpeed == 0)
		previousError = derivative = integral = 0;
}

// lastTickTacho = tickTacho; timeAccumulator = velocitySampleCtr = 0  ???
 
void PilotMotor::Tick(unsigned int eleapsedMs)
{
	timeAccumulator += eleapsedMs;
	if (velocitySampleCtr++ == VELOCITY_SAMPLE_RATE) 
	{
		float newVelocity = (tickTacho - lastTickTacho) * 1000 / timeAccumulator;	// TPS
		velocity = (.25 * newVelocity) + (.75 * velocity);
		timeAccumulator = 0;
		velocitySampleCtr = 0;
		lastTickTacho = tickTacho;
	}

	if (tgtVelocity != 0)
	{
		float pid = Pid(tgtVelocity, velocity, MotorPID.Kp, MotorPID.Ki, MotorPID.Kd,
			previousError, integral, derivative, (float) eleapsedMs / 1000);
		if (velocity == 0)
			power = tgtVelocity >= 0 ? max(KICK_POWER, power) : -max(KICK_POWER, power);		// minimum kick to get motor moving
		else
			power = constrain(power + pid, -100, 100);
	}
	else
		power = 0;
}

//----------------------------------------------------------------------------

float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& integral, float& derivative, float dt)
{
	float error = setPoint - presentValue;
	integral = integral + error * dt;
	float newDerivative = (error - previousError) / (dt * 20); 
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

	// +++ steering previousError = previousIntegral = previousDerivative = 0;

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

bool headingInRange(float h1, float tolerance)
{
	// +++ needs some more thought
	// +++ I still think abs diff of 2 normalized headings should work

	// Stack Overflow answer for [-180,180] 
	// http://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
	//a = targetA - sourceA
	//a += (a>180) ? -360 : (a<-180) ? 360 : 0

	// assumes h1 is normalized [-180,180]
	float a = h1 - H;
	a += (a > PI) ? -TWO_PI : (a < -TWO_PI) ? TWO_PI : 0;
	return abs(a) < tolerance;

	//NormalizeHeading(h1, 0.0, TWO_PI);
	//float minH = h1 - tolerance;
	//float maxH = h1 + tolerance;
	//NormalizeHeading(H, 0.0, TWO_PI);
	//return H >= minH && H <= maxH;
}

void PilotRegulatorTick()
{
	unsigned long now = millis();
	unsigned int tickElapsedTime = now - lastTickTime;

	if (headingStop && headingInRange(headingGoal, (float)(20 * DEG_TO_RAD)))
	{ 
		M1.power *= .5;		// +++ is this working?
		M2.power *= .5;
	}
	else if (headingStop && headingInRange(headingGoal, (float)(5 * DEG_TO_RAD)))
	{
		M1.SetSpeed(0, 0);
		M2.SetSpeed(0, 0);
		headingStop = false;
		MoveCompleteEvent(true);
	}

	M1.tickTacho = M1.GetRawTacho();
	M2.tickTacho = M2.GetRawTacho();

	M1.Tick(tickElapsedTime);
	M2.Tick(tickElapsedTime);

	M1.PinPower(M1.power);
	M2.PinPower(M2.power);

	lastTickTime = now;
}
